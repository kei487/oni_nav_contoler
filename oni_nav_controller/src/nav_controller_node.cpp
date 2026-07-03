#include "oni_nav_controller/nav_controller_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"

using namespace std::chrono_literals;

namespace oni_nav_controller
{

NavControllerNode::NavControllerNode(const rclcpp::NodeOptions & options)
: Node("nav_controller_node", options)
{
  control_rate_hz_ = declare_parameter<double>("control_rate_hz", 20.0);
  bbox_topic_ = declare_parameter<std::string>("bbox_topic", "/perception/bbox");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
  lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  system_state_topic_ = declare_parameter<std::string>("system_state_topic", "/system/state");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  target_lost_topic_ = declare_parameter<std::string>("target_lost_topic", "/nav/target_lost");
  debug_topic_ = declare_parameter<std::string>("debug_topic", "/nav/debug");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_optical_frame");

  StateEstimatorParams ekf_params;
  ekf_params.process_noise_pos = declare_parameter<double>("ekf_process_noise_pos", 0.01);
  ekf_params.process_noise_theta = declare_parameter<double>("ekf_process_noise_theta", 0.01);
  ekf_params.process_noise_vel = declare_parameter<double>("ekf_process_noise_vel", 0.1);
  ekf_params.process_noise_omega = declare_parameter<double>("ekf_process_noise_omega", 0.1);
  ekf_params.meas_noise_vel = declare_parameter<double>("ekf_meas_noise_vel", 0.05);
  ekf_params.meas_noise_omega = declare_parameter<double>("ekf_meas_noise_omega", 0.05);
  state_estimator_ = StateEstimator(ekf_params);

  TargetLocalizerParams target_params;
  target_params.h_thresh = declare_parameter<double>("h_thresh", 120.0);
  target_params.h_hysteresis = declare_parameter<double>("h_hysteresis", 15.0);
  target_params.K_calib = declare_parameter<double>("K_calib", 200.0);
  target_params.image_width = declare_parameter<double>("image_width", 640.0);
  target_params.image_height = declare_parameter<double>("image_height", 480.0);
  target_params.camera_h_fov = declare_parameter<double>("camera_h_fov", 1.0471975511965976);
  target_params.camera_v_fov = declare_parameter<double>("camera_v_fov", 0.7853981633974483);
  target_params.lidar_stable_frames = declare_parameter<int>("lidar_stable_frames", 3);
  target_params.dbscan_eps = declare_parameter<double>("dbscan_eps", 0.15);
  target_params.dbscan_min_pts = declare_parameter<int>("dbscan_min_pts", 5);
  target_params.leg_cluster_min_height = declare_parameter<double>("leg_cluster_min_height", 0.05);
  target_params.leg_cluster_max_height = declare_parameter<double>("leg_cluster_max_height", 1.2);
  target_params.leg_cluster_max_width = declare_parameter<double>("leg_cluster_max_width", 0.8);

  FailsafeParams failsafe_params;
  failsafe_params.target_lost_timeout_ms = declare_parameter<double>("target_lost_timeout_ms", 300.0);
  failsafe_params.use_dead_reckon_on_occlusion =
    declare_parameter<bool>("use_dead_reckon_on_occlusion", true);
  target_localizer_ = TargetLocalizer(target_params, failsafe_params);

  MotionControllerParams motion_params;
  motion_params.decel_rate = declare_parameter<double>("decel_rate", 2.0);
  motion_params.reacquire_spin_rate = declare_parameter<double>("reacquire_spin_rate", 0.3);
  motion_controller_ = MotionController(motion_params);

#if ONI_NAV_WITH_ACADOS
  NmpcParams nmpc_params;
  nmpc_params.horizon_N = declare_parameter<int>("horizon_N", 20);
  nmpc_params.dt = declare_parameter<double>("dt", 0.05);
  nmpc_params.v_max = declare_parameter<double>("v_max", 1.0);
  nmpc_params.omega_max = declare_parameter<double>("omega_max", 2.0);
  nmpc_params.a_max = declare_parameter<double>("a_max", 2.0);
  nmpc_params.alpha_max = declare_parameter<double>("alpha_max", 4.0);
  nmpc_params.d_safe = declare_parameter<double>("d_safe", 0.4);
  nmpc_enabled_ = nmpc_solver_.initialize(nmpc_params);
  if (!nmpc_enabled_) {
    RCLCPP_WARN(get_logger(), "NMPC solver initialization failed");
  }
#endif

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  bbox_sub_ = create_subscription<oni_msgs::msg::BBox>(
    bbox_topic_, rclcpp::SensorDataQoS(),
    std::bind(&NavControllerNode::on_bbox, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&NavControllerNode::on_imu, this, std::placeholders::_1));

  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, rclcpp::SensorDataQoS(),
    std::bind(&NavControllerNode::on_point_cloud, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&NavControllerNode::on_odom, this, std::placeholders::_1));

  system_state_sub_ = create_subscription<oni_msgs::msg::SystemState>(
    system_state_topic_, rclcpp::QoS(10),
    std::bind(&NavControllerNode::on_system_state, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  target_lost_pub_ = create_publisher<std_msgs::msg::Bool>(target_lost_topic_, 10);
  debug_pub_ = create_publisher<oni_msgs::msg::NavDebug>(debug_topic_, 10);

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&NavControllerNode::control_loop, this));

  last_control_time_ = now();

  RCLCPP_INFO(
    get_logger(),
    "NavControllerNode started (Phase 4: integrated) at %.1f Hz",
    control_rate_hz_);
}

void NavControllerNode::on_bbox(const oni_msgs::msg::BBox::SharedPtr msg)
{
  latest_bbox_ = msg;
  has_bbox_ = true;
}

void NavControllerNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  latest_imu_ = msg;
  has_imu_ = true;
}

void NavControllerNode::on_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  latest_lidar_ = msg;
  has_lidar_ = true;
}

void NavControllerNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
  has_odom_ = true;

  if (!state_estimator_.is_initialized()) {
    initialize_state_from_odom(*msg);
  }
}

void NavControllerNode::on_system_state(const oni_msgs::msg::SystemState::SharedPtr msg)
{
  system_state_ = msg->state;
}

void NavControllerNode::control_loop()
{
  ++control_loop_count_;

  const rclcpp::Time current_time = now();
  double dt = (current_time - last_control_time_).seconds();
  last_control_time_ = current_time;

  if (dt <= 0.0 || dt > 1.0) {
    dt = 1.0 / control_rate_hz_;
  }

  if (state_estimator_.is_initialized()) {
    if (has_imu_ && latest_imu_) {
      state_estimator_.predict(dt, imu_from_msg(*latest_imu_));
    }

    if (has_odom_ && latest_odom_) {
      const auto vel = velocity_from_odom(*latest_odom_);

      tf2::Quaternion q;
      tf2::fromMsg(latest_odom_->pose.pose.orientation, q);
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      state_estimator_.update_odom(
        vel,
        latest_odom_->pose.pose.position.x,
        latest_odom_->pose.pose.position.y,
        yaw);
    }
  } else if (has_odom_ && latest_odom_) {
    initialize_state_from_odom(*latest_odom_);
  }

  std::vector<Eigen::Vector3d> lidar_points;
  if (has_lidar_ && latest_lidar_) {
    lidar_points = extract_lidar_points_base(*latest_lidar_);
  }

  BBoxInput bbox;
  if (has_bbox_ && latest_bbox_) {
    bbox = bbox_from_msg(*latest_bbox_);
    const CameraTransform camera_tf = lookup_camera_transform();
    target_localizer_.update(bbox, lidar_points, camera_tf, dt);

    if (target_localizer_.should_notify_lost()) {
      publish_target_lost(true);
      target_lost_latched_ = true;
      RCLCPP_WARN(get_logger(), "Target lost: publishing /nav/target_lost");
    } else if (target_lost_latched_ && target_localizer_.is_target_valid()) {
      publish_target_lost(false);
      target_lost_latched_ = false;
      RCLCPP_INFO(get_logger(), "Target reacquired: clearing /nav/target_lost");
    }
  }

  Velocity3 desired{};
  double debug_cost = 0.0;

#if ONI_NAV_WITH_ACADOS
  if (!target_localizer_.should_stop() &&
    system_state_ == oni_msgs::msg::SystemState::CHASE &&
    target_localizer_.is_target_valid() &&
    state_estimator_.is_initialized() &&
    nmpc_enabled_)
  {
    nmpc_solver_.set_target(target_localizer_.get_target());
    nmpc_solver_.set_obstacles(lidar_points);
    nmpc_solver_.set_initial_state(state_estimator_.get_state());
    if (nmpc_solver_.solve()) {
      desired = nmpc_solver_.get_first_velocity();
      debug_cost = nmpc_solver_.last_cost();
    }
  }
#else
  (void)lidar_points;
#endif

  const auto cmd = motion_controller_.compute(
    desired,
    system_state_,
    target_localizer_.should_stop(),
    dt);

  publish_cmd_vel(cmd.vx, cmd.vy, cmd.omega);
  publish_debug(debug_cost);

  if (control_loop_count_ % static_cast<uint64_t>(control_rate_hz_) == 0) {
    const auto & x = state_estimator_.get_state();
    const auto target = target_localizer_.get_target();
    RCLCPP_DEBUG(
      get_logger(),
      "phase=%d target_valid=%d p=(%.2f, %.2f) ekf_vel=(%.2f, %.2f, %.2f)",
      static_cast<int>(target_localizer_.get_phase()),
      target_localizer_.is_target_valid(),
      target.x, target.y,
      x(VX), x(VY), x(OMEGA));
  }
}

void NavControllerNode::initialize_state_from_odom(const nav_msgs::msg::Odometry & odom)
{
  StateVector x0 = StateVector::Zero();
  x0(X) = odom.pose.pose.position.x;
  x0(Y) = odom.pose.pose.position.y;

  tf2::Quaternion q;
  tf2::fromMsg(odom.pose.pose.orientation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  x0(THETA) = yaw;

  const auto vel = velocity_from_odom(odom);
  x0(VX) = vel.vx;
  x0(VY) = vel.vy;
  x0(OMEGA) = vel.omega;

  state_estimator_.reset(x0);
  RCLCPP_INFO(get_logger(), "StateEstimator initialized from odometry");
}

ImuSample NavControllerNode::imu_from_msg(const sensor_msgs::msg::Imu & imu) const
{
  ImuSample sample;
  sample.ax = imu.linear_acceleration.x;
  sample.ay = imu.linear_acceleration.y;
  sample.omega = imu.angular_velocity.z;
  return sample;
}

Velocity3 NavControllerNode::velocity_from_odom(const nav_msgs::msg::Odometry & odom) const
{
  Velocity3 vel;
  vel.vx = odom.twist.twist.linear.x;
  vel.vy = odom.twist.twist.linear.y;
  vel.omega = odom.twist.twist.angular.z;
  return vel;
}

BBoxInput NavControllerNode::bbox_from_msg(const oni_msgs::msg::BBox & bbox) const
{
  BBoxInput input;
  input.u = bbox.u;
  input.h = bbox.h;
  input.area_ratio = bbox.area_ratio;
  input.lost = bbox.lost;
  return input;
}

std::vector<Eigen::Vector3d> NavControllerNode::extract_lidar_points_base(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<Eigen::Vector3d> points;
  if (cloud.width * cloud.height == 0) {
    return points;
  }

  Eigen::Isometry3d source_to_base = Eigen::Isometry3d::Identity();
  try {
    const auto tf_msg = tf_buffer_->lookupTransform(
      base_frame_, cloud.header.frame_id, cloud.header.stamp, tf2::durationFromSec(0.1));
    tf2::Transform tf_transform;
    tf2::fromMsg(tf_msg.transform, tf_transform);
    const tf2::Matrix3x3 basis = tf_transform.getBasis();
    Eigen::Matrix3d rotation;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        rotation(row, col) = basis[row][col];
      }
    }
    const tf2::Vector3 origin = tf_transform.getOrigin();
    source_to_base.linear() = rotation;
    source_to_base.translation() = Eigen::Vector3d(origin.x(), origin.y(), origin.z());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "LiDAR TF lookup failed: %s", ex.what());
    return points;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  points.reserve(cloud.width * cloud.height);
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }
    const Eigen::Vector3d point_source(*iter_x, *iter_y, *iter_z);
    points.push_back(source_to_base * point_source);
  }

  return points;
}

CameraTransform NavControllerNode::lookup_camera_transform()
{
  CameraTransform camera_tf;
  try {
    const auto tf_msg = tf_buffer_->lookupTransform(
      camera_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
    tf2::Transform tf_transform;
    tf2::fromMsg(tf_msg.transform, tf_transform);
    const tf2::Matrix3x3 basis = tf_transform.getBasis();
    Eigen::Matrix3d rotation;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        rotation(row, col) = basis[row][col];
      }
    }
    const tf2::Vector3 origin = tf_transform.getOrigin();
    camera_tf.base_to_camera.setIdentity();
    camera_tf.base_to_camera.linear() = rotation;
    camera_tf.base_to_camera.translation() = Eigen::Vector3d(origin.x(), origin.y(), origin.z());
    camera_tf.valid = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Camera TF lookup failed: %s", ex.what());
  }
  return camera_tf;
}

void NavControllerNode::publish_cmd_vel(double vx, double vy, double omega)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx;
  cmd.linear.y = vy;
  cmd.angular.z = omega;
  cmd_vel_pub_->publish(cmd);
}

void NavControllerNode::publish_target_lost(bool lost)
{
  std_msgs::msg::Bool msg;
  msg.data = lost;
  target_lost_pub_->publish(msg);
}

void NavControllerNode::publish_debug(double cost)
{
  oni_msgs::msg::NavDebug debug;
  debug.stamp = now();
  debug.phase = (target_localizer_.get_phase() == TargetPhase::B) ?
    oni_msgs::msg::NavDebug::PHASE_B :
    oni_msgs::msg::NavDebug::PHASE_A;

  const auto target = target_localizer_.get_target();
  debug.p_target = {target.x, target.y};

  if (state_estimator_.is_initialized()) {
    const auto & x = state_estimator_.get_state();
    debug.x_est = {x(X), x(Y), x(THETA), x(VX), x(VY), x(OMEGA)};
  } else {
    debug.x_est = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  debug.cost = cost;
  debug.nav_status = static_cast<uint8_t>(motion_controller_.status());
  debug_pub_->publish(debug);
}

}  // namespace oni_nav_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<oni_nav_controller::NavControllerNode>());
  rclcpp::shutdown();
  return 0;
}

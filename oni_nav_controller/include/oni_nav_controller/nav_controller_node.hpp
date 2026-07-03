#ifndef ONI_NAV_CONTROLLER__NAV_CONTROLLER_NODE_HPP_
#define ONI_NAV_CONTROLLER__NAV_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "oni_msgs/msg/b_box.hpp"
#include "oni_msgs/msg/nav_debug.hpp"
#include "oni_msgs/msg/system_state.hpp"
#include "oni_nav_controller/motion_controller.hpp"
#include "oni_nav_controller/state_estimator.hpp"
#include "oni_nav_controller/target_localizer.hpp"
#include "oni_nav_controller/types.hpp"
#if ONI_NAV_WITH_ACADOS
#include "oni_nav_controller/nmpc_solver.hpp"
#endif
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace oni_nav_controller
{

class NavControllerNode : public rclcpp::Node
{
public:
  explicit NavControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_bbox(const oni_msgs::msg::BBox::SharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_system_state(const oni_msgs::msg::SystemState::SharedPtr msg);
  void control_loop();
  void publish_cmd_vel(double vx, double vy, double omega);
  void publish_debug(double cost = 0.0);
  void publish_target_lost(bool lost);
  void initialize_state_from_odom(const nav_msgs::msg::Odometry & odom);
  ImuSample imu_from_msg(const sensor_msgs::msg::Imu & imu) const;
  Velocity3 velocity_from_odom(const nav_msgs::msg::Odometry & odom) const;
  BBoxInput bbox_from_msg(const oni_msgs::msg::BBox & bbox) const;
  std::vector<Eigen::Vector3d> extract_lidar_points_base(
    const sensor_msgs::msg::PointCloud2 & cloud);
  CameraTransform lookup_camera_transform();

  // Parameters
  double control_rate_hz_{20.0};
  std::string bbox_topic_;
  std::string imu_topic_;
  std::string lidar_topic_;
  std::string odom_topic_;
  std::string system_state_topic_;
  std::string cmd_vel_topic_;
  std::string target_lost_topic_;
  std::string debug_topic_;
  std::string base_frame_{"base_link"};
  std::string camera_frame_{"camera_optical_frame"};

  // Subscribers
  rclcpp::Subscription<oni_msgs::msg::BBox>::SharedPtr bbox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<oni_msgs::msg::SystemState>::SharedPtr system_state_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_lost_pub_;
  rclcpp::Publisher<oni_msgs::msg::NavDebug>::SharedPtr debug_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  bool has_bbox_{false};
  bool has_imu_{false};
  bool has_lidar_{false};
  bool has_odom_{false};
  uint8_t system_state_{oni_msgs::msg::SystemState::IDLE};

  oni_msgs::msg::BBox::SharedPtr latest_bbox_;
  sensor_msgs::msg::Imu::SharedPtr latest_imu_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_lidar_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  uint64_t control_loop_count_{0};

  StateEstimator state_estimator_;
  TargetLocalizer target_localizer_;
  MotionController motion_controller_;
  bool target_lost_latched_{false};
#if ONI_NAV_WITH_ACADOS
  NmpcSolver nmpc_solver_;
  bool nmpc_enabled_{false};
#endif
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace oni_nav_controller

#endif  // ONI_NAV_CONTROLLER__NAV_CONTROLLER_NODE_HPP_

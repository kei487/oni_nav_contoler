#ifndef ONI_NAV_CONTROLLER__TYPES_HPP_
#define ONI_NAV_CONTROLLER__TYPES_HPP_

#include <Eigen/Dense>

namespace oni_nav_controller
{

constexpr int STATE_DIM = 6;
constexpr int ODOM_MEAS_DIM = 3;

enum StateIndex : int
{
  X = 0,
  Y = 1,
  THETA = 2,
  VX = 3,
  VY = 4,
  OMEGA = 5
};

using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
using OdomMeasurement = Eigen::Matrix<double, ODOM_MEAS_DIM, 1>;

struct ImuSample
{
  double ax{0.0};
  double ay{0.0};
  double omega{0.0};
};

struct Velocity3
{
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

struct StateEstimatorParams
{
  double process_noise_pos{0.01};
  double process_noise_theta{0.01};
  double process_noise_vel{0.1};
  double process_noise_omega{0.1};
  double meas_noise_vel{0.05};
  double meas_noise_omega{0.05};
};

enum class TargetPhase
{
  A = 0,
  B = 1
};

struct Target2D
{
  double x{0.0};
  double y{0.0};
  bool valid{false};
};

struct BBoxInput
{
  double u{0.0};
  double h{0.0};
  double area_ratio{0.0};
  bool lost{false};
};

struct TargetLocalizerParams
{
  double h_thresh{120.0};
  double h_hysteresis{15.0};
  double K_calib{200.0};
  double image_width{640.0};
  double image_height{480.0};
  double camera_h_fov{1.0471975511965976};
  double camera_v_fov{0.7853981633974483};
  int lidar_stable_frames{3};
  double dbscan_eps{0.15};
  int dbscan_min_pts{5};
  double leg_cluster_min_height{0.05};
  double leg_cluster_max_height{1.2};
  double leg_cluster_max_width{0.8};
};

struct FailsafeParams
{
  double target_lost_timeout_ms{300.0};
  bool use_dead_reckon_on_occlusion{true};
};

struct CameraTransform
{
  Eigen::Isometry3d base_to_camera{Eigen::Isometry3d::Identity()};
  bool valid{false};
};

struct NmpcParams
{
  int horizon_N{20};
  double dt{0.05};
  double v_max{1.0};
  double omega_max{2.0};
  double a_max{2.0};
  double alpha_max{4.0};
  double d_safe{0.4};
  double target_exclude_radius{0.5};
};

enum class NavStatus : uint8_t
{
  IDLE = 0,
  CHASE = 1,
  DECEL = 2,
  REACQUIRE = 3,
  STOPPED = 4
};

struct MotionControllerParams
{
  double decel_rate{2.0};
  double reacquire_spin_rate{0.3};
};

}  // namespace oni_nav_controller

#endif  // ONI_NAV_CONTROLLER__TYPES_HPP_

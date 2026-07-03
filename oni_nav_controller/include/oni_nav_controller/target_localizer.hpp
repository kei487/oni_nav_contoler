#ifndef ONI_NAV_CONTROLLER__TARGET_LOCALIZER_HPP_
#define ONI_NAV_CONTROLLER__TARGET_LOCALIZER_HPP_

#include <vector>

#include "oni_nav_controller/failsafe.hpp"
#include "oni_nav_controller/phase_fsm.hpp"
#include "oni_nav_controller/types.hpp"

namespace oni_nav_controller
{

class TargetLocalizer
{
public:
  TargetLocalizer(
    const TargetLocalizerParams & params = TargetLocalizerParams{},
    const FailsafeParams & failsafe_params = FailsafeParams{});

  void update(
    const BBoxInput & bbox,
    const std::vector<Eigen::Vector3d> & lidar_points_base,
    const CameraTransform & camera_tf,
    double dt);

  Target2D get_target() const;
  TargetPhase get_phase() const;
  bool is_target_valid() const;
  bool frustum_has_points() const;
  bool should_stop() const;
  bool should_notify_lost() const;
  OcclusionMode occlusion_mode() const;

  static double bbox_to_bearing(double u, double image_width, double h_fov);
  static double height_to_distance(double h, double k_calib);
  static Target2D polar_to_target(double distance, double bearing);

private:
  Target2D compute_phase_a(const BBoxInput & bbox) const;
  Target2D compute_phase_b(
    const BBoxInput & bbox,
    const std::vector<Eigen::Vector3d> & lidar_points_base,
    const CameraTransform & camera_tf,
    bool & frustum_has_points) const;

  std::vector<Eigen::Vector3d> filter_frustum(
    const BBoxInput & bbox,
    const std::vector<Eigen::Vector3d> & lidar_points_base,
    const CameraTransform & camera_tf) const;

  std::vector<std::vector<Eigen::Vector3d>> dbscan(
    const std::vector<Eigen::Vector3d> & points) const;

  Target2D cluster_to_target(const std::vector<std::vector<Eigen::Vector3d>> & clusters) const;

  TargetLocalizerParams params_;
  PhaseFsm phase_fsm_;
  Failsafe failsafe_;
  Target2D target_;
  Target2D last_valid_target_;
  int lidar_valid_streak_{0};
  bool frustum_has_points_{false};
  BBoxInput last_bbox_;
  bool has_last_bbox_{false};

  bool compute_frustum_has_points(
    const BBoxInput & bbox,
    const std::vector<Eigen::Vector3d> & lidar_points_base,
    const CameraTransform & camera_tf) const;
};

}  // namespace oni_nav_controller

#endif  // ONI_NAV_CONTROLLER__TARGET_LOCALIZER_HPP_

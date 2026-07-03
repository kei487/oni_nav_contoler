#ifndef ONI_NAV_CONTROLLER__FAILSAFE_HPP_
#define ONI_NAV_CONTROLLER__FAILSAFE_HPP_

#include "oni_nav_controller/types.hpp"

namespace oni_nav_controller
{

enum class OcclusionMode
{
  NONE = 0,
  FALLBACK_PHASE_A = 1,
  DEAD_RECKON = 2
};

class Failsafe
{
public:
  explicit Failsafe(const FailsafeParams & params = FailsafeParams{});

  void update(bool bbox_lost, bool frustum_empty, double dt_ms);
  void on_lidar_occlusion(bool bbox_available);
  void reset_lost_timer();

  bool should_stop() const;
  bool should_notify_lost() const;
  OcclusionMode occlusion_mode() const;

  Target2D dead_reckon_target(const Target2D & last_target, double dt) const;
  void set_target_velocity(double vx, double vy);

private:
  FailsafeParams params_;
  double lost_accumulated_ms_{0.0};
  bool target_lost_active_{false};
  bool notify_lost_{false};
  OcclusionMode occlusion_mode_{OcclusionMode::NONE};
  double target_vel_x_{0.0};
  double target_vel_y_{0.0};
};

}  // namespace oni_nav_controller

#endif  // ONI_NAV_CONTROLLER__FAILSAFE_HPP_

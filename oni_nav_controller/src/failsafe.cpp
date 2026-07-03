#include "oni_nav_controller/failsafe.hpp"

namespace oni_nav_controller
{

Failsafe::Failsafe(const FailsafeParams & params)
: params_(params)
{
}

void Failsafe::update(bool bbox_lost, bool frustum_empty, double dt_ms)
{
  notify_lost_ = false;

  if (bbox_lost && frustum_empty) {
    lost_accumulated_ms_ += dt_ms;
  } else {
    lost_accumulated_ms_ = 0.0;
    target_lost_active_ = false;
  }

  if (lost_accumulated_ms_ >= params_.target_lost_timeout_ms) {
    target_lost_active_ = true;
    notify_lost_ = true;
    occlusion_mode_ = OcclusionMode::NONE;
  }
}

void Failsafe::on_lidar_occlusion(bool bbox_available)
{
  if (!bbox_available) {
    occlusion_mode_ = OcclusionMode::NONE;
    return;
  }

  occlusion_mode_ = params_.use_dead_reckon_on_occlusion ?
    OcclusionMode::DEAD_RECKON :
    OcclusionMode::FALLBACK_PHASE_A;
}

void Failsafe::reset_lost_timer()
{
  lost_accumulated_ms_ = 0.0;
  target_lost_active_ = false;
  notify_lost_ = false;
}

bool Failsafe::should_stop() const
{
  return target_lost_active_;
}

bool Failsafe::should_notify_lost() const
{
  return notify_lost_;
}

OcclusionMode Failsafe::occlusion_mode() const
{
  return occlusion_mode_;
}

Target2D Failsafe::dead_reckon_target(const Target2D & last_target, double dt) const
{
  Target2D predicted = last_target;
  if (!last_target.valid) {
    return predicted;
  }

  predicted.x = last_target.x + target_vel_x_ * dt;
  predicted.y = last_target.y + target_vel_y_ * dt;
  predicted.valid = true;
  return predicted;
}

void Failsafe::set_target_velocity(double vx, double vy)
{
  target_vel_x_ = vx;
  target_vel_y_ = vy;
}

}  // namespace oni_nav_controller

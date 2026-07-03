#include "oni_nav_controller/phase_fsm.hpp"

namespace oni_nav_controller
{

PhaseFsm::PhaseFsm(const TargetLocalizerParams & params)
: params_(params)
{
}

void PhaseFsm::update(double bbox_height, bool lidar_valid)
{
  const double enter_b = params_.h_thresh + params_.h_hysteresis;
  const double exit_b = params_.h_thresh - params_.h_hysteresis;

  if (phase_ == TargetPhase::A) {
    if (bbox_height >= enter_b && lidar_valid) {
      phase_ = TargetPhase::B;
    }
    return;
  }

  if (bbox_height < exit_b || !lidar_valid) {
    phase_ = TargetPhase::A;
  }
}

TargetPhase PhaseFsm::get_phase() const
{
  return phase_;
}

}  // namespace oni_nav_controller

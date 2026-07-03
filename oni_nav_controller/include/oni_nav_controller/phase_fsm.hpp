#ifndef ONI_NAV_CONTROLLER__PHASE_FSM_HPP_
#define ONI_NAV_CONTROLLER__PHASE_FSM_HPP_

#include "oni_nav_controller/types.hpp"

namespace oni_nav_controller
{

class PhaseFsm
{
public:
  explicit PhaseFsm(const TargetLocalizerParams & params = TargetLocalizerParams{});

  void update(double bbox_height, bool lidar_valid);
  TargetPhase get_phase() const;

private:
  TargetLocalizerParams params_;
  TargetPhase phase_{TargetPhase::A};
};

}  // namespace oni_nav_controller

#endif  // ONI_NAV_CONTROLLER__PHASE_FSM_HPP_

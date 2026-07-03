#ifndef ONI_NAV_CONTROLLER__MOTION_CONTROLLER_HPP_
#define ONI_NAV_CONTROLLER__MOTION_CONTROLLER_HPP_

#include <cstdint>

#include "oni_nav_controller/types.hpp"

namespace oni_nav_controller
{

class MotionController
{
public:
  explicit MotionController(const MotionControllerParams & params = MotionControllerParams{});

  Velocity3 compute(
    const Velocity3 & desired,
    uint8_t system_state,
    bool emergency_stop,
    double dt);

  NavStatus status() const;
  const Velocity3 & last_command() const;

private:
  static double approach_zero(double value, double max_delta);

  MotionControllerParams params_;
  Velocity3 last_cmd_;
  NavStatus status_{NavStatus::IDLE};
};

}  // namespace oni_nav_controller

#endif  // ONI_NAV_CONTROLLER__MOTION_CONTROLLER_HPP_

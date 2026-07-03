#include "oni_nav_controller/motion_controller.hpp"

#include <algorithm>
#include <cmath>

namespace oni_nav_controller
{

namespace
{
constexpr uint8_t kSystemIdle = 0;
constexpr uint8_t kSystemChase = 1;
constexpr uint8_t kSystemLost = 2;
constexpr uint8_t kSystemStop = 3;
}  // namespace

MotionController::MotionController(const MotionControllerParams & params)
: params_(params)
{
}

Velocity3 MotionController::compute(
  const Velocity3 & desired,
  uint8_t system_state,
  bool emergency_stop,
  double dt)
{
  const double max_delta = std::max(params_.decel_rate * dt, 1e-6);

  if (system_state == kSystemStop) {
    status_ = NavStatus::STOPPED;
    last_cmd_.vx = approach_zero(last_cmd_.vx, max_delta);
    last_cmd_.vy = approach_zero(last_cmd_.vy, max_delta);
    last_cmd_.omega = approach_zero(last_cmd_.omega, max_delta);
    return last_cmd_;
  }

  if (emergency_stop) {
    status_ = NavStatus::DECEL;
    last_cmd_.vx = approach_zero(last_cmd_.vx, max_delta);
    last_cmd_.vy = approach_zero(last_cmd_.vy, max_delta);
    last_cmd_.omega = approach_zero(last_cmd_.omega, max_delta);
    return last_cmd_;
  }

  if (system_state == kSystemLost) {
    status_ = NavStatus::REACQUIRE;
    last_cmd_.vx = approach_zero(last_cmd_.vx, max_delta);
    last_cmd_.vy = approach_zero(last_cmd_.vy, max_delta);
    last_cmd_.omega = params_.reacquire_spin_rate;
    return last_cmd_;
  }

  if (system_state == kSystemIdle) {
    status_ = NavStatus::IDLE;
    last_cmd_ = {0.0, 0.0, 0.0};
    return last_cmd_;
  }

  if (system_state == kSystemChase) {
    status_ = NavStatus::CHASE;
    last_cmd_.vx = desired.vx;
    last_cmd_.vy = desired.vy;
    last_cmd_.omega = desired.omega;
    return last_cmd_;
  }

  status_ = NavStatus::IDLE;
  last_cmd_ = {0.0, 0.0, 0.0};
  return last_cmd_;
}

NavStatus MotionController::status() const
{
  return status_;
}

const Velocity3 & MotionController::last_command() const
{
  return last_cmd_;
}

double MotionController::approach_zero(double value, double max_delta)
{
  if (std::abs(value) <= max_delta) {
    return 0.0;
  }
  return value - std::copysign(max_delta, value);
}

}  // namespace oni_nav_controller

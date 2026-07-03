#ifndef ONI_NAV_CONTROLLER__STATE_ESTIMATOR_HPP_
#define ONI_NAV_CONTROLLER__STATE_ESTIMATOR_HPP_

#include "oni_nav_controller/types.hpp"

namespace oni_nav_controller
{

class StateEstimator
{
public:
  explicit StateEstimator(const StateEstimatorParams & params = StateEstimatorParams{});

  void reset(const StateVector & x0);
  void predict(double dt, const ImuSample & imu);
  void update_odom(const Velocity3 & v_odom);
  void update_odom(const Velocity3 & v_odom, double x, double y, double theta);

  const StateVector & get_state() const;
  bool is_initialized() const;

private:
  static double normalize_angle(double theta);

  StateEstimatorParams params_;
  StateVector x_{StateVector::Zero()};
  StateMatrix P_{StateMatrix::Identity()};
  bool initialized_{false};
};

}  // namespace oni_nav_controller

#endif  // ONI_NAV_CONTROLLER__STATE_ESTIMATOR_HPP_

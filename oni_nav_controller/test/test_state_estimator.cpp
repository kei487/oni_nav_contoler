#include <cmath>

#include "gtest/gtest.h"
#include "oni_nav_controller/state_estimator.hpp"

using oni_nav_controller::ImuSample;
using oni_nav_controller::StateEstimator;
using oni_nav_controller::StateEstimatorParams;
using oni_nav_controller::StateVector;
using oni_nav_controller::THETA;
using oni_nav_controller::Velocity3;
using oni_nav_controller::VX;
using oni_nav_controller::VY;
using oni_nav_controller::X;
using oni_nav_controller::Y;

namespace
{

StateEstimatorParams low_noise_params()
{
  StateEstimatorParams p;
  p.process_noise_pos = 0.001;
  p.process_noise_theta = 0.001;
  p.process_noise_vel = 0.001;
  p.process_noise_omega = 0.001;
  p.meas_noise_vel = 0.001;
  p.meas_noise_omega = 0.001;
  return p;
}

}  // namespace

TEST(StateEstimatorTest, ResetInitializesState)
{
  StateEstimator estimator;
  EXPECT_FALSE(estimator.is_initialized());

  StateVector x0 = StateVector::Zero();
  x0(VX) = 1.0;
  estimator.reset(x0);

  EXPECT_TRUE(estimator.is_initialized());
  EXPECT_DOUBLE_EQ(estimator.get_state()(VX), 1.0);
}

TEST(StateEstimatorTest, PredictAdvancesPositionWithConstantVelocity)
{
  StateEstimator estimator(low_noise_params());

  StateVector x0 = StateVector::Zero();
  x0(VX) = 1.0;
  estimator.reset(x0);

  ImuSample imu;
  const double dt = 0.05;
  for (int i = 0; i < 20; ++i) {
    estimator.predict(dt, imu);
  }

  const auto & x = estimator.get_state();
  EXPECT_NEAR(x(X), 1.0, 0.05);
  EXPECT_NEAR(x(Y), 0.0, 0.05);
  EXPECT_NEAR(x(VX), 1.0, 0.05);
}

TEST(StateEstimatorTest, PredictIntegratesImuAcceleration)
{
  StateEstimator estimator(low_noise_params());
  estimator.reset(StateVector::Zero());

  ImuSample imu;
  imu.ax = 2.0;
  const double dt = 0.1;
  estimator.predict(dt, imu);

  EXPECT_NEAR(estimator.get_state()(VX), 0.2, 0.02);
}

TEST(StateEstimatorTest, UpdateOdomPullsVelocityTowardMeasurement)
{
  StateEstimator estimator;
  StateVector x0 = StateVector::Zero();
  x0(VX) = 0.0;
  estimator.reset(x0);

  Velocity3 odom;
  odom.vx = 1.0;
  odom.vy = 0.5;
  odom.omega = 0.2;
  estimator.update_odom(odom);

  const auto & x = estimator.get_state();
  EXPECT_GT(x(VX), 0.5);
  EXPECT_GT(x(VY), 0.2);
  EXPECT_GT(x(oni_nav_controller::OMEGA), 0.1);
}

TEST(StateEstimatorTest, HolonomicMotionInYDirection)
{
  StateEstimator estimator(low_noise_params());

  StateVector x0 = StateVector::Zero();
  x0(VY) = 0.5;
  estimator.reset(x0);

  ImuSample imu;
  const double dt = 0.05;
  for (int i = 0; i < 20; ++i) {
    estimator.predict(dt, imu);
  }

  EXPECT_NEAR(estimator.get_state()(Y), 0.5, 0.05);
  EXPECT_NEAR(estimator.get_state()(X), 0.0, 0.05);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

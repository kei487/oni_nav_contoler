#include "gtest/gtest.h"
#include "oni_nav_controller/motion_controller.hpp"

using oni_nav_controller::MotionController;
using oni_nav_controller::MotionControllerParams;
using oni_nav_controller::NavStatus;
using oni_nav_controller::Velocity3;

TEST(MotionControllerTest, IdleStateZerosVelocity)
{
  MotionController controller;
  const auto cmd = controller.compute({1.0, 0.0, 0.5}, 0, false, 0.05);
  EXPECT_DOUBLE_EQ(cmd.vx, 0.0);
  EXPECT_EQ(controller.status(), NavStatus::IDLE);
}

TEST(MotionControllerTest, ChaseStateUsesDesiredVelocity)
{
  MotionController controller;
  Velocity3 desired;
  desired.vx = 0.8;
  desired.vy = 0.1;
  desired.omega = 0.2;
  const auto cmd = controller.compute(desired, 1, false, 0.05);
  EXPECT_DOUBLE_EQ(cmd.vx, 0.8);
  EXPECT_DOUBLE_EQ(cmd.vy, 0.1);
  EXPECT_DOUBLE_EQ(cmd.omega, 0.2);
  EXPECT_EQ(controller.status(), NavStatus::CHASE);
}

TEST(MotionControllerTest, EmergencyStopDecelerates)
{
  MotionControllerParams params;
  params.decel_rate = 2.0;
  MotionController controller(params);

  controller.compute({1.0, 0.0, 0.0}, 1, false, 0.05);
  const auto cmd = controller.compute({1.0, 0.0, 0.0}, 1, true, 0.05);
  EXPECT_LT(std::abs(cmd.vx), 1.0);
  EXPECT_EQ(controller.status(), NavStatus::DECEL);
}

TEST(MotionControllerTest, LostStateSpinsInPlace)
{
  MotionControllerParams params;
  params.reacquire_spin_rate = 0.3;
  MotionController controller(params);
  const auto cmd = controller.compute({0.0, 0.0, 0.0}, 2, false, 0.05);
  EXPECT_DOUBLE_EQ(cmd.omega, 0.3);
  EXPECT_EQ(controller.status(), NavStatus::REACQUIRE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

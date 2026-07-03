#include "gtest/gtest.h"
#include "oni_nav_controller/nmpc_solver.hpp"

using oni_nav_controller::NmpcParams;
using oni_nav_controller::NmpcSolver;
using oni_nav_controller::StateVector;
using oni_nav_controller::Target2D;
using oni_nav_controller::VX;
using oni_nav_controller::VY;
using oni_nav_controller::OMEGA;

TEST(NmpcSolverTest, SolvesTowardTarget)
{
  NmpcParams params;
  NmpcSolver solver;
  ASSERT_TRUE(solver.initialize(params));

  StateVector x0 = StateVector::Zero();
  solver.set_initial_state(x0);

  Target2D target;
  target.x = 2.0;
  target.y = 0.0;
  target.valid = true;
  solver.set_target(target);
  solver.set_obstacles({});

  ASSERT_TRUE(solver.solve());
  const auto vel = solver.get_first_velocity();
  EXPECT_GT(vel.vx, 0.0);
}

TEST(NmpcSolverTest, RejectsInvalidTarget)
{
  NmpcSolver solver;
  ASSERT_TRUE(solver.initialize(NmpcParams{}));
  solver.set_initial_state(StateVector::Zero());

  Target2D target;
  target.valid = false;
  solver.set_target(target);

  EXPECT_FALSE(solver.solve());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

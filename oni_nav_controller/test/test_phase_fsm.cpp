#include "gtest/gtest.h"
#include "oni_nav_controller/phase_fsm.hpp"

using oni_nav_controller::PhaseFsm;
using oni_nav_controller::TargetLocalizerParams;
using oni_nav_controller::TargetPhase;

TEST(PhaseFsmTest, StartsInPhaseA)
{
  PhaseFsm fsm;
  EXPECT_EQ(fsm.get_phase(), TargetPhase::A);
}

TEST(PhaseFsmTest, TransitionsToPhaseBWithHysteresis)
{
  TargetLocalizerParams params;
  params.h_thresh = 100.0;
  params.h_hysteresis = 10.0;
  PhaseFsm fsm(params);

  fsm.update(90.0, true);
  EXPECT_EQ(fsm.get_phase(), TargetPhase::A);

  fsm.update(111.0, true);
  EXPECT_EQ(fsm.get_phase(), TargetPhase::B);
}

TEST(PhaseFsmTest, FallsBackToPhaseAWhenHeightDrops)
{
  TargetLocalizerParams params;
  params.h_thresh = 100.0;
  params.h_hysteresis = 10.0;
  PhaseFsm fsm(params);

  fsm.update(120.0, true);
  EXPECT_EQ(fsm.get_phase(), TargetPhase::B);

  fsm.update(85.0, true);
  EXPECT_EQ(fsm.get_phase(), TargetPhase::A);
}

TEST(PhaseFsmTest, RequiresLidarValidForPhaseB)
{
  TargetLocalizerParams params;
  params.h_thresh = 100.0;
  params.h_hysteresis = 10.0;
  PhaseFsm fsm(params);

  fsm.update(120.0, false);
  EXPECT_EQ(fsm.get_phase(), TargetPhase::A);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

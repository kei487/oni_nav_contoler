#include <vector>

#include "gtest/gtest.h"
#include "oni_nav_controller/target_localizer.hpp"

using oni_nav_controller::BBoxInput;
using oni_nav_controller::CameraTransform;
using oni_nav_controller::FailsafeParams;
using oni_nav_controller::TargetLocalizer;
using oni_nav_controller::TargetLocalizerParams;
using oni_nav_controller::TargetPhase;

namespace
{

TargetLocalizerParams default_params()
{
  TargetLocalizerParams params;
  params.h_thresh = 100.0;
  params.h_hysteresis = 10.0;
  params.K_calib = 200.0;
  params.image_width = 640.0;
  params.image_height = 480.0;
  params.lidar_stable_frames = 1;
  params.dbscan_eps = 0.3;
  params.dbscan_min_pts = 3;
  params.leg_cluster_min_height = 0.01;
  return params;
}

}  // namespace

TEST(TargetLocalizerTest, PhaseAComputesTargetFromBBox)
{
  TargetLocalizer localizer(default_params());

  BBoxInput bbox;
  bbox.u = 320.0;
  bbox.h = 50.0;
  bbox.area_ratio = 0.1;
  bbox.lost = false;

  CameraTransform tf;
  tf.valid = false;
  localizer.update(bbox, {}, tf, 0.05);

  const auto target = localizer.get_target();
  EXPECT_TRUE(target.valid);
  EXPECT_NEAR(target.x, 4.0, 0.1);
  EXPECT_NEAR(target.y, 0.0, 0.1);
  EXPECT_EQ(localizer.get_phase(), TargetPhase::A);
}

TEST(TargetLocalizerTest, PhaseBUsesLidarCluster)
{
  TargetLocalizerParams params = default_params();
  params.h_thresh = 50.0;
  TargetLocalizer localizer(params);

  BBoxInput bbox;
  bbox.u = 320.0;
  bbox.h = 120.0;
  bbox.area_ratio = 0.2;
  bbox.lost = false;

  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 8; ++i) {
    points.emplace_back(2.0 + 0.02 * i, 0.05 * i, 0.1);
  }

  CameraTransform tf;
  tf.valid = true;
  Eigen::Matrix3d rotation;
  rotation << 0.0, -1.0, 0.0,
    0.0, 0.0, -1.0,
    1.0, 0.0, 0.0;
  tf.base_to_camera.linear() = rotation;
  localizer.update(bbox, points, tf, 0.05);

  const auto target = localizer.get_target();
  EXPECT_TRUE(target.valid);
  EXPECT_NEAR(target.x, 2.0, 0.5);
  EXPECT_NEAR(target.y, 0.0, 0.5);
}

TEST(TargetLocalizerTest, TargetLostTriggersStop)
{
  FailsafeParams failsafe_params;
  failsafe_params.target_lost_timeout_ms = 100.0;
  TargetLocalizer localizer(default_params(), failsafe_params);

  BBoxInput bbox;
  bbox.lost = true;
  CameraTransform tf;

  for (int i = 0; i < 7; ++i) {
    localizer.update(bbox, {}, tf, 0.05);
  }

  EXPECT_TRUE(localizer.should_stop());
  EXPECT_FALSE(localizer.is_target_valid());
}

TEST(TargetLocalizerTest, BboxToBearingCenterIsZero)
{
  const double bearing = TargetLocalizer::bbox_to_bearing(320.0, 640.0, 1.047);
  EXPECT_NEAR(bearing, 0.0, 1e-3);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

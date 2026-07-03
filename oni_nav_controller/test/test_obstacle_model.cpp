#include "gtest/gtest.h"
#include "oni_nav_controller/obstacle_model.hpp"

using oni_nav_controller::NmpcParams;
using oni_nav_controller::ObstacleModel;
using oni_nav_controller::Target2D;

TEST(ObstacleModelTest, ExcludesPointsNearTarget)
{
  ObstacleModel model;
  Target2D target;
  target.x = 2.0;
  target.y = 0.0;
  target.valid = true;

  std::vector<Eigen::Vector3d> points;
  points.emplace_back(2.1, 0.0, 0.5);
  points.emplace_back(5.0, 0.0, 0.5);

  const auto obstacles = model.build_obstacle_set(points, target);
  EXPECT_EQ(obstacles.size(), 1u);
  EXPECT_NEAR(obstacles.front().x(), 5.0, 1e-6);
}

TEST(ObstacleModelTest, MinDistanceFindsNearestPoint)
{
  ObstacleModel model;
  std::vector<Eigen::Vector2d> obstacles;
  obstacles.emplace_back(3.0, 0.0);
  obstacles.emplace_back(0.0, 4.0);

  const double dist = model.min_distance(0.0, 0.0, obstacles);
  EXPECT_NEAR(dist, 3.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

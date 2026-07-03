#ifndef ONI_NAV_CONTROLLER__OBSTACLE_MODEL_HPP_
#define ONI_NAV_CONTROLLER__OBSTACLE_MODEL_HPP_

#include <vector>

#include <Eigen/Dense>

#include "oni_nav_controller/types.hpp"

namespace oni_nav_controller
{

class ObstacleModel
{
public:
  explicit ObstacleModel(const NmpcParams & params = NmpcParams{});

  std::vector<Eigen::Vector2d> build_obstacle_set(
    const std::vector<Eigen::Vector3d> & lidar_points_base,
    const Target2D & target) const;

  double min_distance(double px, double py, const std::vector<Eigen::Vector2d> & obstacles) const;
  double barrier_weighted_distance(double distance) const;

private:
  NmpcParams params_;
};

}  // namespace oni_nav_controller

#endif  // ONI_NAV_CONTROLLER__OBSTACLE_MODEL_HPP_

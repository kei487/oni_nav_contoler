#include "oni_nav_controller/obstacle_model.hpp"

#include <cmath>
#include <limits>

namespace oni_nav_controller
{

ObstacleModel::ObstacleModel(const NmpcParams & params)
: params_(params)
{
}

std::vector<Eigen::Vector2d> ObstacleModel::build_obstacle_set(
  const std::vector<Eigen::Vector3d> & lidar_points_base,
  const Target2D & target) const
{
  std::vector<Eigen::Vector2d> obstacles;
  obstacles.reserve(lidar_points_base.size());

  for (const auto & point : lidar_points_base) {
    if (target.valid) {
      const double dx = point.x() - target.x;
      const double dy = point.y() - target.y;
      if (std::hypot(dx, dy) < params_.target_exclude_radius) {
        continue;
      }
    }
    obstacles.emplace_back(point.x(), point.y());
  }

  return obstacles;
}

double ObstacleModel::min_distance(
  double px, double py,
  const std::vector<Eigen::Vector2d> & obstacles) const
{
  if (obstacles.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double min_dist = std::numeric_limits<double>::infinity();
  for (const auto & obs : obstacles) {
    const double dist = std::hypot(px - obs.x(), py - obs.y());
    min_dist = std::min(min_dist, dist);
  }
  return min_dist;
}

double ObstacleModel::barrier_weighted_distance(double distance) const
{
  if (!std::isfinite(distance)) {
    return params_.d_safe + 1.0;
  }
  return distance;
}

}  // namespace oni_nav_controller

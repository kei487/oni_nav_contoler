#include "oni_nav_controller/target_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace oni_nav_controller
{

namespace
{

double clamp_min(double value, double min_value)
{
  return std::max(value, min_value);
}

}  // namespace

TargetLocalizer::TargetLocalizer(
  const TargetLocalizerParams & params,
  const FailsafeParams & failsafe_params)
: params_(params),
  phase_fsm_(params),
  failsafe_(failsafe_params)
{
}

void TargetLocalizer::update(
  const BBoxInput & bbox,
  const std::vector<Eigen::Vector3d> & lidar_points_base,
  const CameraTransform & camera_tf,
  double dt)
{
  frustum_has_points_ = false;
  const double dt_ms = dt * 1000.0;

  if (bbox.lost || bbox.h <= 0.0) {
    target_.valid = false;
    lidar_valid_streak_ = 0;
    phase_fsm_.update(bbox.h, false);

    if (camera_tf.valid && has_last_bbox_) {
      frustum_has_points_ = compute_frustum_has_points(
        last_bbox_, lidar_points_base, camera_tf);
    }

    failsafe_.update(true, !frustum_has_points_, dt_ms);
    if (failsafe_.should_stop()) {
      target_.valid = false;
    }
    return;
  }

  last_bbox_ = bbox;
  has_last_bbox_ = true;

  bool frustum_points = false;
  Target2D phase_b_target;
  if (camera_tf.valid) {
    phase_b_target = compute_phase_b(bbox, lidar_points_base, camera_tf, frustum_points);
    frustum_has_points_ = frustum_points;
  }

  const bool lidar_valid = phase_b_target.valid;
  if (lidar_valid) {
    ++lidar_valid_streak_;
  } else {
    lidar_valid_streak_ = 0;
  }

  const bool lidar_stable = lidar_valid_streak_ >= params_.lidar_stable_frames;
  phase_fsm_.update(bbox.h, lidar_stable);

  const auto phase = phase_fsm_.get_phase();
  if (phase == TargetPhase::B && lidar_valid) {
    target_ = phase_b_target;
  } else if (phase == TargetPhase::B && !lidar_valid && last_valid_target_.valid) {
    failsafe_.on_lidar_occlusion(true);
    if (failsafe_.occlusion_mode() == OcclusionMode::DEAD_RECKON) {
      target_ = failsafe_.dead_reckon_target(last_valid_target_, dt);
    } else {
      target_ = compute_phase_a(bbox);
    }
  } else {
    failsafe_.on_lidar_occlusion(false);
    target_ = compute_phase_a(bbox);
  }

  if (target_.valid && last_valid_target_.valid && dt > 0.0) {
    failsafe_.set_target_velocity(
      (target_.x - last_valid_target_.x) / dt,
      (target_.y - last_valid_target_.y) / dt);
  }

  if (target_.valid) {
    last_valid_target_ = target_;
    failsafe_.reset_lost_timer();
  }

  failsafe_.update(false, !frustum_has_points_, dt_ms);

  if (failsafe_.should_stop()) {
    target_.valid = false;
  }
}

Target2D TargetLocalizer::get_target() const
{
  return target_;
}

TargetPhase TargetLocalizer::get_phase() const
{
  return phase_fsm_.get_phase();
}

bool TargetLocalizer::is_target_valid() const
{
  return target_.valid && !failsafe_.should_stop();
}

bool TargetLocalizer::frustum_has_points() const
{
  return frustum_has_points_;
}

bool TargetLocalizer::should_stop() const
{
  return failsafe_.should_stop();
}

bool TargetLocalizer::should_notify_lost() const
{
  return failsafe_.should_notify_lost();
}

OcclusionMode TargetLocalizer::occlusion_mode() const
{
  return failsafe_.occlusion_mode();
}

double TargetLocalizer::bbox_to_bearing(double u, double image_width, double h_fov)
{
  const double cx = image_width * 0.5;
  const double fx = (image_width * 0.5) / std::tan(h_fov * 0.5);
  return std::atan2(u - cx, fx);
}

double TargetLocalizer::height_to_distance(double h, double k_calib)
{
  return k_calib / clamp_min(h, 1.0);
}

Target2D TargetLocalizer::polar_to_target(double distance, double bearing)
{
  Target2D target;
  target.x = distance * std::cos(bearing);
  target.y = distance * std::sin(bearing);
  target.valid = distance > 0.0;
  return target;
}

Target2D TargetLocalizer::compute_phase_a(const BBoxInput & bbox) const
{
  const double bearing = bbox_to_bearing(bbox.u, params_.image_width, params_.camera_h_fov);
  const double distance = height_to_distance(bbox.h, params_.K_calib);
  return polar_to_target(distance, bearing);
}

Target2D TargetLocalizer::compute_phase_b(
  const BBoxInput & bbox,
  const std::vector<Eigen::Vector3d> & lidar_points_base,
  const CameraTransform & camera_tf,
  bool & frustum_has_points) const
{
  frustum_has_points = false;
  const auto filtered = filter_frustum(bbox, lidar_points_base, camera_tf);
  if (filtered.empty()) {
    return {};
  }

  frustum_has_points = true;
  const auto clusters = dbscan(filtered);
  return cluster_to_target(clusters);
}

std::vector<Eigen::Vector3d> TargetLocalizer::filter_frustum(
  const BBoxInput & bbox,
  const std::vector<Eigen::Vector3d> & lidar_points_base,
  const CameraTransform & camera_tf) const
{
  const double cx = params_.image_width * 0.5;
  const double cy = params_.image_height * 0.5;
  const double fx = (params_.image_width * 0.5) / std::tan(params_.camera_h_fov * 0.5);
  const double fy = (params_.image_height * 0.5) / std::tan(params_.camera_v_fov * 0.5);

  const double area_px = bbox.area_ratio * params_.image_width * params_.image_height;
  const double bbox_w = area_px / clamp_min(bbox.h, 1.0);
  const double u_min = bbox.u - bbox_w * 0.5;
  const double u_max = bbox.u + bbox_w * 0.5;
  const double v_min = cy - bbox.h * 0.5;
  const double v_max = cy + bbox.h * 0.5;

  const double yaw_min = std::atan((u_min - cx) / fx);
  const double yaw_max = std::atan((u_max - cx) / fx);
  const double pitch_min = std::atan((v_min - cy) / fy);
  const double pitch_max = std::atan((v_max - cy) / fy);

  std::vector<Eigen::Vector3d> filtered;
  filtered.reserve(lidar_points_base.size());

  for (const auto & point_base : lidar_points_base) {
    const Eigen::Vector3d point_cam = camera_tf.base_to_camera * point_base;
    if (point_cam.z() <= 0.1) {
      continue;
    }

    const double yaw = std::atan2(point_cam.x(), point_cam.z());
    const double pitch = std::atan2(point_cam.y(), point_cam.z());
    if (yaw >= yaw_min && yaw <= yaw_max && pitch >= pitch_min && pitch <= pitch_max) {
      filtered.push_back(point_base);
    }
  }

  return filtered;
}

std::vector<std::vector<Eigen::Vector3d>> TargetLocalizer::dbscan(
  const std::vector<Eigen::Vector3d> & points) const
{
  const int n = static_cast<int>(points.size());
  if (n == 0) {
    return {};
  }

  const int undefined = -2;
  const int noise = -1;
  std::vector<int> labels(n, undefined);

  auto region_query = [&](int idx) {
    std::vector<int> neighbors;
    for (int j = 0; j < n; ++j) {
      if ((points[idx] - points[j]).norm() <= params_.dbscan_eps) {
        neighbors.push_back(j);
      }
    }
    return neighbors;
  };

  int cluster_id = 0;
  for (int i = 0; i < n; ++i) {
    if (labels[i] != undefined) {
      continue;
    }

    auto neighbors = region_query(i);
    if (static_cast<int>(neighbors.size()) < params_.dbscan_min_pts) {
      labels[i] = noise;
      continue;
    }

    labels[i] = cluster_id;
    std::vector<int> seeds = neighbors;
    for (size_t seed_idx = 0; seed_idx < seeds.size(); ++seed_idx) {
      const int current = seeds[seed_idx];
      if (labels[current] == noise) {
        labels[current] = cluster_id;
      }
      if (labels[current] != undefined) {
        continue;
      }

      labels[current] = cluster_id;
      auto current_neighbors = region_query(current);
      if (static_cast<int>(current_neighbors.size()) >= params_.dbscan_min_pts) {
        seeds.insert(seeds.end(), current_neighbors.begin(), current_neighbors.end());
      }
    }
    ++cluster_id;
  }

  std::vector<std::vector<Eigen::Vector3d>> clusters(cluster_id);
  for (int i = 0; i < n; ++i) {
    if (labels[i] >= 0) {
      clusters[labels[i]].push_back(points[i]);
    }
  }

  clusters.erase(
    std::remove_if(
      clusters.begin(), clusters.end(),
      [](const std::vector<Eigen::Vector3d> & cluster) {
        return cluster.empty();
      }),
    clusters.end());

  return clusters;
}

Target2D TargetLocalizer::cluster_to_target(
  const std::vector<std::vector<Eigen::Vector3d>> & clusters) const
{
  double best_score = std::numeric_limits<double>::max();
  Target2D best_target;

  for (const auto & cluster : clusters) {
    if (static_cast<int>(cluster.size()) < params_.dbscan_min_pts) {
      continue;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d min_pt = cluster.front();
    Eigen::Vector3d max_pt = cluster.front();
    for (const auto & pt : cluster) {
      centroid += pt;
      min_pt = min_pt.cwiseMin(pt);
      max_pt = max_pt.cwiseMax(pt);
    }
    centroid /= static_cast<double>(cluster.size());

    const double height = max_pt.z() - min_pt.z();
    const double width = std::max(max_pt.x() - min_pt.x(), max_pt.y() - min_pt.y());
    if (height < params_.leg_cluster_min_height || height > params_.leg_cluster_max_height) {
      continue;
    }
    if (width > params_.leg_cluster_max_width) {
      continue;
    }

    const double range_xy = std::hypot(centroid.x(), centroid.y());
    if (range_xy < best_score) {
      best_score = range_xy;
      best_target.x = centroid.x();
      best_target.y = centroid.y();
      best_target.valid = true;
    }
  }

  return best_target;
}

bool TargetLocalizer::compute_frustum_has_points(
  const BBoxInput & bbox,
  const std::vector<Eigen::Vector3d> & lidar_points_base,
  const CameraTransform & camera_tf) const
{
  return !filter_frustum(bbox, lidar_points_base, camera_tf).empty();
}

}  // namespace oni_nav_controller

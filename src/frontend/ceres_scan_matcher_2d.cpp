// Copyright 2026 zwc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_slam/frontend/ceres_scan_matcher_2d.hpp"

#include <ceres/ceres.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>

namespace simple_slam
{
namespace
{
pcl::PointCloud<pcl::PointXYZ>::Ptr PointsToPclCloud(const std::vector<Point2D> & points)
{
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(points.size());
  for (const auto & point : points) {
    cloud->push_back(
      pcl::PointXYZ(
        static_cast<float>(point.x),
        static_cast<float>(point.y),
        0.0F));
  }
  return cloud;
}

struct PointPair
{
  Point2D source;
  Point2D target;
};

struct PointToPointResidual
{
  PointToPointResidual(const Point2D & source_point, const Point2D & target_point)
  : source_point_(source_point), target_point_(target_point)
  {
  }
  template<typename T>
  bool operator()(const T * const pose, T * residuals) const
  {
    const T & tx = pose[0];
    const T & ty = pose[1];
    const T & yaw = pose[2];

    const T cos_yaw = ceres::cos(yaw);
    const T sin_yaw = ceres::sin(yaw);

    const T source_x = T(source_point_.x);
    const T source_y = T(source_point_.y);
    const T target_x = T(target_point_.x);
    const T target_y = T(target_point_.y);

    const T transformed_x = cos_yaw * source_x - sin_yaw * source_y + tx;
    const T transformed_y = sin_yaw * source_x + cos_yaw * source_y + ty;

    residuals[0] = transformed_x - target_x;
    residuals[1] = transformed_y - target_y;
    return true;
  }
  Point2D source_point_;
  Point2D target_point_;
};
}  // namespace

CeresScanMatcher2D::CeresScanMatcher2D(Options options)
: options_(options)
{
}

Pose2D CeresScanMatcher2D::Match(
  const std::vector<Point2D> & current_points,
  const std::vector<Point2D> & previous_points,
  const Pose2D & initial_relative_pose) const
{
  if (current_points.empty() || previous_points.empty()) {
    return initial_relative_pose;
  }
  auto target_cloud = PointsToPclCloud(previous_points);
  pcl::KdTreeFLANN<pcl::PointXYZ> target_kdtree;
  target_kdtree.setInputCloud(target_cloud);
  std::vector<PointPair> correspondences;
  correspondences.reserve(current_points.size());
  std::vector<int> nearest_indices(1);
  std::vector<float> nearest_squared_distances(1);
  const double max_distance = std::max(options_.max_correspondence_distance, 1e-6);
  const double max_distance_sq = max_distance * max_distance;
  for (const auto & source_point : current_points) {
    const Point2D predicted_point = TransformPoint(source_point, initial_relative_pose);

    pcl::PointXYZ query(
      static_cast<float>(predicted_point.x),
      static_cast<float>(predicted_point.y),
      0.0F);

    if (target_kdtree.nearestKSearch(query, 1, nearest_indices, nearest_squared_distances) <= 0) {
      continue;
    }
    if (static_cast<double>(nearest_squared_distances[0]) > max_distance_sq) {
      continue;
    }
    correspondences.push_back(
      PointPair{source_point,
        previous_points[static_cast<size_t>(nearest_indices[0])]});
  }
  if (correspondences.size() < 4) {
    return initial_relative_pose;
  }

  double pose[3] = {
    initial_relative_pose.x,
    initial_relative_pose.y,
    initial_relative_pose.yaw
  };
  ceres::Problem problem;
  for (const auto & pair : correspondences) {
    auto * cost_function = new ceres::AutoDiffCostFunction<PointToPointResidual, 2, 3>(
      new PointToPointResidual(pair.source, pair.target));

    ceres::LossFunction * loss_function = nullptr;
    if (options_.huber_scale > 0.0) {
      loss_function = new ceres::HuberLoss(options_.huber_scale);
    }
    problem.AddResidualBlock(cost_function, loss_function, pose);
  }
  ceres::Solver::Options solver_options;
  solver_options.max_num_iterations = std::max(options_.max_num_iterations, 1);
  solver_options.linear_solver_type = ceres::DENSE_QR;
  solver_options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  if (!summary.IsSolutionUsable()) {
    return initial_relative_pose;
  }

  Pose2D matched_pose;
  matched_pose.x = pose[0];
  matched_pose.y = pose[1];
  matched_pose.yaw = NormalizeAngle(pose[2]);
  return matched_pose;
}

}  // namespace simple_slam

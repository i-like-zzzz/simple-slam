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

#include "simple_slam/frontend/local_slam_frontend.hpp"

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "rcutils/logging_macros.h"
#include "simple_slam/frontend/ceres_scan_matcher_2d.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace simple_slam {

namespace {

Eigen::Matrix4f PoseToEigenTransform(const Pose2D& pose) {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0, 0) = static_cast<float>(std::cos(pose.yaw));
  transform(0, 1) = static_cast<float>(-std::sin(pose.yaw));
  transform(1, 0) = static_cast<float>(std::sin(pose.yaw));
  transform(1, 1) = static_cast<float>(std::cos(pose.yaw));
  transform(0, 3) = static_cast<float>(pose.x);
  transform(1, 3) = static_cast<float>(pose.y);
  return transform;
}

Pose2D EigenTransformToPose(const Eigen::Matrix4f& transform) {
  Pose2D pose;
  pose.x = static_cast<double>(transform(0, 3));
  pose.y = static_cast<double>(transform(1, 3));
  pose.yaw = NormalizeAngle(std::atan2(transform(1, 0), transform(0, 0)));
  return pose;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointsToPointCloud(
    const std::vector<Point2D>& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(points.size());
  for (const auto& point : points) {
    cloud->push_back(pcl::PointXYZ(static_cast<float>(point.x),
                                   static_cast<float>(point.y), 0.0F));
  }
  return cloud;
}

const char* MatcherTypeToString(const LidarOdomMatcherType matcher_type) {
  switch (matcher_type) {
    case LidarOdomMatcherType::kGeneralizedIcp:
      return "generalized_icp";
    case LidarOdomMatcherType::kCorrelative:
      return "correlative";
    case LidarOdomMatcherType::kCeres:
      return "ceres";
    case LidarOdomMatcherType::kPointToPointIcp:
    default:
      return "point_to_point_icp";
  }
}

}  // namespace

LocalSlamFrontend::LocalSlamFrontend(Options options) : options_(options) {
  // 子图插入数量上限直接复用前端活动子图配置，避免两边参数漂移。
  options_.submap.num_range_data_limit = options_.active_submap_num_range_data;
}

LocalSlamResult2D LocalSlamFrontend::AddScan(
    const sensor_msgs::msg::LaserScan& scan,
    const nav_msgs::msg::Odometry* odom_msg) {
  LocalSlamResult2D result;
  // 这一帧进来之后，前端的主要工作都在这里串起来：
  // 先清理点云，再给出预测位姿，然后做匹配，最后判断要不要插关键帧。
  result.range_data = VoxelFilter(FilterScan(scan));
  // TODO(zwc): 后面可以再补一层离群点剔除。
  // 点太少时这一帧信息量不够，继续做 ICP 或子图匹配意义不大，先直接跳过。
  if (static_cast<int>(result.range_data.returns.size()) <
      options_.min_range_points_for_match) {
    return result;
  }
  // 激光里程计的预测位姿，先用外部里程计增量，如果没有外部里程计再用激光里程计增量。
  const Pose2D predicted_pose = PredictPose(odom_msg);
  Pose2D lidar_odom_pose = predicted_pose;
  if (odom_msg == nullptr && has_previous_range_data_) {
    // 没有外部里程计时，先用相邻两帧做一次ICP，估一个大致运动。
    const Pose2D relative_lidar_motion =
        MatchToPreviousScan(result.range_data, lidar_odom_delta_);
    lidar_odom_delta_ = relative_lidar_motion;
    lidar_odom_pose =
        ComposePoses(lidar_odom_pose_estimate_, relative_lidar_motion);
  }

  MaybeGrowActiveSubmaps(predicted_pose);
  result.local_pose = MatchToActiveSubmap(result.range_data, lidar_odom_pose);

  ++accumulated_scans_;
  result.valid = true;
  result.is_keyframe = accumulated_scans_ >= options_.scans_per_accumulation &&
                       ShouldCreateKeyframe(result.local_pose);
  result.insertion_required = result.is_keyframe && options_.enable_map_update;

  if (result.insertion_required) {
    result.insertion_submap_ids = InsertIntoActiveSubmaps(result.range_data, result.local_pose);
    last_keyframe_pose_ = result.local_pose;
    has_last_keyframe_pose_ = true;
    accumulated_scans_ = 0;
  }

  if (has_pose_estimate_) {
    // 如果外部里程计可用，就顺手把当前局部位姿变化记下来，
    // 这样后面即便 /odom 暂时断掉，也还有一个最近的运动增量可以接着用。
    if (odom_msg != nullptr) {
      lidar_odom_delta_ = RelativePose(local_pose_estimate_, result.local_pose);
    }
  }
  local_pose_estimate_ = result.local_pose;
  lidar_odom_pose_estimate_ = lidar_odom_pose;
  has_pose_estimate_ = true;
  previous_range_data_ = result.range_data;
  has_previous_range_data_ = true;
  return result;
}

const std::vector<std::shared_ptr<Submap2D>>&
LocalSlamFrontend::GetActiveSubmaps() const {
  return active_submaps_;
}

const std::vector<std::shared_ptr<Submap2D>>&
LocalSlamFrontend::GetFinishedSubmaps() const {
  return finished_submaps_;
}

const Pose2D& LocalSlamFrontend::lidar_odom_pose() const {
  return lidar_odom_pose_estimate_;
}

RangeData2D LocalSlamFrontend::FilterScan(
    const sensor_msgs::msg::LaserScan& scan) const {
  RangeData2D data;
  data.stamp = scan.header.stamp;

  double angle = scan.angle_min;
  for (const float range : scan.ranges) {
    if (std::isfinite(range) && range >= options_.min_range &&
        range <= options_.max_range) {
      data.returns.push_back(
          Point2D{static_cast<double>(range) * std::cos(angle),
                  static_cast<double>(range) * std::sin(angle)});
    }
    angle += scan.angle_increment;
  }

  return data;
}
// 使用栅格来优化点云，把点云打在同一个栅格的点云进行滤除。
RangeData2D LocalSlamFrontend::VoxelFilter(
    const RangeData2D& range_data) const {
  if (options_.voxel_filter_size <= 0.0) {
    return range_data;
  }

  // 当前这里不是严格的体素质心滤波，而是把落在同一格子里的重复点合并掉。
  RangeData2D filtered;
  filtered.stamp = range_data.stamp;
  std::vector<Point2D> sorted_points = range_data.returns;
  // 先按格子编号排序，这样同一个格子的点会排到一起。
  std::sort(
      sorted_points.begin(), sorted_points.end(),
      [this](const Point2D& lhs, const Point2D& rhs) {
        const int lhs_x =
            static_cast<int>(std::floor(lhs.x / options_.voxel_filter_size));
        const int lhs_y =
            static_cast<int>(std::floor(lhs.y / options_.voxel_filter_size));
        const int rhs_x =
            static_cast<int>(std::floor(rhs.x / options_.voxel_filter_size));
        const int rhs_y =
            static_cast<int>(std::floor(rhs.y / options_.voxel_filter_size));
        if (lhs_x != rhs_x) {
          return lhs_x < rhs_x;
        }
        return lhs_y < rhs_y;
      });

  int last_x = std::numeric_limits<int>::min();
  int last_y = std::numeric_limits<int>::min();
  for (const auto& point : sorted_points) {
    const int cell_x =
        static_cast<int>(std::floor(point.x / options_.voxel_filter_size));
    const int cell_y =
        static_cast<int>(std::floor(point.y / options_.voxel_filter_size));
    if (cell_x == last_x && cell_y == last_y) {
      continue;
    }
    filtered.returns.push_back(point);
    last_x = cell_x;
    last_y = cell_y;
  }
  return filtered;
}
// 降采样点云，控制最大点云数量，避免ICP计算量过大。
std::vector<Point2D> LocalSlamFrontend::DownsamplePoints(
    const std::vector<Point2D>& points, const int max_points) const {
  if (max_points <= 0 || static_cast<int>(points.size()) <= max_points) {
    return points;
  }

  std::vector<Point2D> sampled_points;
  sampled_points.reserve(static_cast<size_t>(max_points));
  const size_t stride =
      std::max<size_t>(1, points.size() / static_cast<size_t>(max_points));
  for (size_t index = 0; index < points.size() &&
                         static_cast<int>(sampled_points.size()) < max_points;
       index += stride) {
    sampled_points.push_back(points[index]);
  }
  return sampled_points;
}

Pose2D LocalSlamFrontend::PoseFromOdom(
    const nav_msgs::msg::Odometry& odom_msg) const {
  Pose2D odom_pose;
  odom_pose.x = odom_msg.pose.pose.position.x;
  odom_pose.y = odom_msg.pose.pose.position.y;

  tf2::Quaternion q;
  tf2::fromMsg(odom_msg.pose.pose.orientation, q);
  odom_pose.yaw = tf2::getYaw(q);
  return odom_pose;
}

Pose2D LocalSlamFrontend::PredictPose(const nav_msgs::msg::Odometry* odom_msg) {
  // 为当前帧匹配生成初值：优先用 /odom 增量，其次用激光增量。
  if (!has_pose_estimate_) {
    if (odom_msg != nullptr) {
      previous_odom_pose_ = PoseFromOdom(*odom_msg);
      has_previous_odom_ = true;
      return previous_odom_pose_;
    }
    return Pose2D{};
  }

  if (odom_msg != nullptr) {
    const Pose2D current_odom_pose = PoseFromOdom(*odom_msg);
    if (!has_previous_odom_) {
      // /odom 中途接入时，先缓存一帧作为增量基准。
      previous_odom_pose_ = current_odom_pose;
      has_previous_odom_ = true;
      return local_pose_estimate_;
    }

    const Pose2D odom_delta =
        RelativePose(previous_odom_pose_, current_odom_pose);
    previous_odom_pose_ = current_odom_pose;
    return ComposePoses(local_pose_estimate_, odom_delta);
  }
  if (!has_previous_range_data_) {
    return local_pose_estimate_;
  }

  // /odom 缺失时，退回到最近一次激光匹配得到的相对运动。
  return ComposePoses(local_pose_estimate_, lidar_odom_delta_);
}

Pose2D LocalSlamFrontend::MatchToPreviousScan(
    const RangeData2D& current_range_data,
    const Pose2D& initial_relative_pose) const {
  if (!has_previous_range_data_ || previous_range_data_.returns.empty()) {
    return initial_relative_pose;
  }

  const auto current_points = DownsamplePoints(current_range_data.returns,
                                               options_.lidar_odom_max_points);
  const auto previous_points = DownsamplePoints(previous_range_data_.returns,
                                                options_.lidar_odom_max_points);
  if (current_points.empty() || previous_points.empty()) {
    return initial_relative_pose;
  }
  // 选择不同的匹配方式，默认使用 icp 进行匹配。
  const auto match_start_time = std::chrono::steady_clock::now();
  Pose2D matched_pose;
  switch (options_.lidar_odom_matcher) {
    case LidarOdomMatcherType::kGeneralizedIcp:
      matched_pose = MatchToPreviousScanGeneralizedIcp(
          current_points, previous_points, initial_relative_pose);
      break;
    case LidarOdomMatcherType::kCorrelative:
      matched_pose = MatchToPreviousScanCorrelative(
          current_points, previous_points, initial_relative_pose);
      break;
    case LidarOdomMatcherType::kCeres:
      matched_pose = MatchToPreviousScanCeres(current_points, previous_points,
                                              initial_relative_pose);
      break;
    case LidarOdomMatcherType::kPointToPointIcp:
    default:
      matched_pose = MatchToPreviousScanPointToPointIcp(
          current_points, previous_points, initial_relative_pose);
      break;
  }

  const auto match_end_time = std::chrono::steady_clock::now();
  const double match_time_ms = std::chrono::duration<double, std::milli>(
                                   match_end_time - match_start_time)
                                   .count();
  RCUTILS_LOG_INFO_NAMED("simple_slam_frontend",
                         "lidar_odom_matcher=%s match_time_ms=%.3f "
                         "current_points=%zu previous_points=%zu",
                         MatcherTypeToString(options_.lidar_odom_matcher),
                         match_time_ms, current_points.size(),
                         previous_points.size());
  return matched_pose;
}

Pose2D LocalSlamFrontend::MatchToPreviousScanPointToPointIcp(
    const std::vector<Point2D>& current_points,
    const std::vector<Point2D>& previous_points,
    const Pose2D& initial_relative_pose) const {
  auto source_cloud = PointsToPointCloud(current_points);
  auto target_cloud = PointsToPointCloud(previous_points);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source_cloud);
  icp.setInputTarget(target_cloud);
  // ICP 返回的是“把当前帧点坐标变到上一帧点坐标系”的变换。
  // 对于激光观测到的静态环境点，这个量正好等于传感器从上一帧到当前帧的运动增量。
  icp.setMaximumIterations(options_.lidar_odom_max_iterations);
  icp.setMaxCorrespondenceDistance(options_.lidar_odom_point_sigma);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACOutlierRejectionThreshold(options_.lidar_odom_point_sigma);

  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  icp.align(aligned_cloud, PoseToEigenTransform(initial_relative_pose));
  if (!icp.hasConverged()) {
    return initial_relative_pose;
  }
  return EigenTransformToPose(icp.getFinalTransformation());
}

Pose2D LocalSlamFrontend::MatchToPreviousScanGeneralizedIcp(
    const std::vector<Point2D>& current_points,
    const std::vector<Point2D>& previous_points,
    const Pose2D& initial_relative_pose) const {
  auto source_cloud = PointsToPointCloud(current_points);
  auto target_cloud = PointsToPointCloud(previous_points);

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setInputSource(source_cloud);
  gicp.setInputTarget(target_cloud);
  gicp.setMaximumIterations(options_.lidar_odom_max_iterations);
  gicp.setMaxCorrespondenceDistance(options_.lidar_odom_point_sigma);
  gicp.setTransformationEpsilon(1e-6);
  gicp.setEuclideanFitnessEpsilon(1e-6);
  gicp.setRANSACOutlierRejectionThreshold(options_.lidar_odom_point_sigma);

  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  gicp.align(aligned_cloud, PoseToEigenTransform(initial_relative_pose));
  if (!gicp.hasConverged()) {
    return initial_relative_pose;
  }
  return EigenTransformToPose(gicp.getFinalTransformation());
}

Pose2D LocalSlamFrontend::MatchToPreviousScanCorrelative(
    const std::vector<Point2D>& current_points,
    const std::vector<Point2D>& previous_points,
    const Pose2D& initial_relative_pose) const {
  if (options_.lidar_odom_linear_window <= 0.0 ||
      options_.lidar_odom_angular_window <= 0.0) {
    return initial_relative_pose;
  }
  if (options_.scan_matcher.linear_step <= 0.0 ||
      options_.scan_matcher.angular_step <= 0.0) {
    return initial_relative_pose;
  }

  Pose2D best_pose = initial_relative_pose;
  double best_score =
      ScoreScanToScanCandidate(current_points, previous_points,
                               initial_relative_pose, initial_relative_pose);

  for (double yaw_delta = -options_.lidar_odom_angular_window;
       yaw_delta <= options_.lidar_odom_angular_window + 1e-6;
       yaw_delta += options_.scan_matcher.angular_step) {
    for (double dx = -options_.lidar_odom_linear_window;
         dx <= options_.lidar_odom_linear_window + 1e-6;
         dx += options_.scan_matcher.linear_step) {
      for (double dy = -options_.lidar_odom_linear_window;
           dy <= options_.lidar_odom_linear_window + 1e-6;
           dy += options_.scan_matcher.linear_step) {
        Pose2D candidate = initial_relative_pose;
        candidate.x += dx;
        candidate.y += dy;
        candidate.yaw = NormalizeAngle(candidate.yaw + yaw_delta);

        const double score = ScoreScanToScanCandidate(
            current_points, previous_points, candidate, initial_relative_pose);
        if (score > best_score) {
          best_score = score;
          best_pose = candidate;
        }
      }
    }
  }

  return best_pose;
}

Pose2D LocalSlamFrontend::MatchToPreviousScanCeres(
    const std::vector<Point2D>& current_points,
    const std::vector<Point2D>& previous_points,
    const Pose2D& initial_relative_pose) const {
  CeresScanMatcher2D::Options matcher_options;
  matcher_options.huber_scale = options_.lidar_odom_ceres_huber_scale;
  matcher_options.max_correspondence_distance =
      options_.lidar_odom_ceres_max_correspondence_distance;
  matcher_options.max_num_iterations =
      options_.lidar_odom_ceres_max_num_iterations;
  CeresScanMatcher2D matcher(matcher_options);
  return matcher.Match(current_points, previous_points, initial_relative_pose);
}

double LocalSlamFrontend::ScoreScanToScanCandidate(
    const std::vector<Point2D>& current_points,
    const std::vector<Point2D>& previous_points,
    const Pose2D& candidate_relative_pose,
    const Pose2D& initial_relative_pose) const {
  if (current_points.empty() || previous_points.empty()) {
    return std::numeric_limits<double>::lowest();
  }

  const double sigma = std::max(options_.lidar_odom_point_sigma, 1e-3);
  const double sigma_sq = sigma * sigma;
  double score = 0.0;

  for (const auto& point_in_current : current_points) {
    const Point2D point_in_previous =
        TransformPoint(point_in_current, candidate_relative_pose);
    double min_distance_sq = std::numeric_limits<double>::max();
    for (const auto& previous_point : previous_points) {
      const double dx = point_in_previous.x - previous_point.x;
      const double dy = point_in_previous.y - previous_point.y;
      min_distance_sq = std::min(min_distance_sq, dx * dx + dy * dy);
    }

    score += std::exp(-0.5 * min_distance_sq / sigma_sq);
  }

  score /= static_cast<double>(current_points.size());

  const double translation_penalty =
      std::hypot(candidate_relative_pose.x - initial_relative_pose.x,
                 candidate_relative_pose.y - initial_relative_pose.y);
  const double rotation_penalty = std::abs(
      NormalizeAngle(candidate_relative_pose.yaw - initial_relative_pose.yaw));

  return score - options_.lidar_odom_translation_weight * translation_penalty -
         options_.lidar_odom_rotation_weight * rotation_penalty;
}

Pose2D LocalSlamFrontend::MatchToActiveSubmap(
    const RangeData2D& range_data, const Pose2D& predicted_pose) const {
  if (active_submaps_.empty()) {
    return predicted_pose;
  }

  const auto& matching_submap = active_submaps_.back();
  if (!matching_submap->HasSufficientData()) {
    return predicted_pose;
  }

  // 这里先做实时相关匹配，保证前端不依赖后端也能独立收敛。
  Pose2D best_pose = predicted_pose;
  double best_score = ScoreCandidate(*matching_submap, range_data,
                                     predicted_pose, predicted_pose);

  for (double yaw_delta = -options_.scan_matcher.angular_window;
       yaw_delta <= options_.scan_matcher.angular_window + 1e-6;
       yaw_delta += options_.scan_matcher.angular_step) {
    for (double dx = -options_.scan_matcher.linear_window;
         dx <= options_.scan_matcher.linear_window + 1e-6;
         dx += options_.scan_matcher.linear_step) {
      for (double dy = -options_.scan_matcher.linear_window;
           dy <= options_.scan_matcher.linear_window + 1e-6;
           dy += options_.scan_matcher.linear_step) {
        Pose2D candidate = predicted_pose;
        candidate.x += dx;
        candidate.y += dy;
        candidate.yaw = NormalizeAngle(candidate.yaw + yaw_delta);

        const double score = ScoreCandidate(*matching_submap, range_data,
                                            candidate, predicted_pose);
        if (score > best_score) {
          best_score = score;
          best_pose = candidate;
        }
      }
    }
  }

  return best_pose;
}

double LocalSlamFrontend::ScoreCandidate(const Submap2D& submap,
                                         const RangeData2D& range_data,
                                         const Pose2D& candidate_pose,
                                         const Pose2D& predicted_pose) const {
  double score = 0.0;
  for (const auto& hit_in_sensor : range_data.returns) {
    const Point2D hit_in_world = TransformPoint(hit_in_sensor, candidate_pose);
    score += submap.GetProbability(hit_in_world);
  }

  score /= static_cast<double>(range_data.returns.size());

  const double translation_penalty = std::hypot(
      candidate_pose.x - predicted_pose.x, candidate_pose.y - predicted_pose.y);
  const double rotation_penalty =
      std::abs(NormalizeAngle(candidate_pose.yaw - predicted_pose.yaw));

  return score -
         options_.scan_matcher.translation_delta_cost_weight *
             translation_penalty -
         options_.scan_matcher.rotation_delta_cost_weight * rotation_penalty;
}

bool LocalSlamFrontend::ShouldCreateKeyframe(const Pose2D& matched_pose) const {
  if (!has_last_keyframe_pose_) {
    return true;
  }

  const double translation = std::hypot(matched_pose.x - last_keyframe_pose_.x,
                                        matched_pose.y - last_keyframe_pose_.y);
  const double rotation =
      std::abs(NormalizeAngle(matched_pose.yaw - last_keyframe_pose_.yaw));
  return translation >= options_.keyframe_translation_threshold ||
         rotation >= options_.keyframe_rotation_threshold;
}

std::vector<int> LocalSlamFrontend::InsertIntoActiveSubmaps(const RangeData2D& range_data,
                                                const Pose2D& matched_pose) {
  MaybeGrowActiveSubmaps(matched_pose);
  std::vector<int> insertion_submap_ids;
  insertion_submap_ids.reserve(active_submaps_.size());
  for (auto& submap : active_submaps_) {
    if (!submap->IsFinished()) {
      submap->InsertRangeData(range_data, matched_pose);
      insertion_submap_ids.push_back(submap->id());
    }
  }
  // 如果第一个子图完成了，就把它移到 finished_submaps_
  if (active_submaps_.size() == 2 && active_submaps_.front()->IsFinished()) {
    finished_submaps_.push_back(active_submaps_.front());
    active_submaps_.erase(active_submaps_.begin());
  }
  return insertion_submap_ids;
}
// 维护两个重叠活动子图的生命周期，保证总有一个子图在生长，另一个子图在成熟。
// 如果活跃子图为空先创建子图
// 如果当前活跃子图为1并且当前子图的插入数量达到一半，就创建第二个子图
void LocalSlamFrontend::MaybeGrowActiveSubmaps(const Pose2D& matched_pose) {
  if (active_submaps_.empty()) {
    active_submaps_.push_back(std::make_shared<Submap2D>(
        next_submap_id_++, options_.submap, matched_pose));
    return;
  }

  if (!options_.enable_map_update) {
    return;
  }

  if (active_submaps_.size() == 1 &&
      active_submaps_.back()->num_insertions() >=
          options_.active_submap_num_range_data / 2) {
    active_submaps_.push_back(std::make_shared<Submap2D>(
        next_submap_id_++, options_.submap, matched_pose));
  }
}

}  // namespace simple_slam

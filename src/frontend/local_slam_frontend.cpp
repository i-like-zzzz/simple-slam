#include "simple_slam/frontend/local_slam_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace simple_slam
{

LocalSlamFrontend::LocalSlamFrontend(Options options)
: options_(options)
{
  options_.submap.num_range_data_limit = options_.active_submap_num_range_data;
}

LocalSlamResult2D LocalSlamFrontend::AddScan(
  const sensor_msgs::msg::LaserScan & scan,
  const nav_msgs::msg::Odometry * odom_msg)
{
  LocalSlamResult2D result;
  // 前端入口统一在这里完成：预处理、位姿预测、局部匹配、关键帧判定。
  result.range_data = VoxelFilter(FilterScan(scan));

  if (static_cast<int>(result.range_data.returns.size()) < options_.min_range_points_for_match) {
    return result;
  }

  const Pose2D predicted_pose = PredictPose(odom_msg);
  Pose2D lidar_odom_pose = predicted_pose;
  if (odom_msg == nullptr && has_previous_range_data_) {
    // 没有外部里程计时，基于相邻两帧直接估计相对运动。
    const Pose2D relative_lidar_motion = MatchToPreviousScan(result.range_data, lidar_odom_delta_);
    lidar_odom_delta_ = relative_lidar_motion;
    lidar_odom_pose = ComposePoses(lidar_odom_pose_estimate_, relative_lidar_motion);
  }
  MaybeGrowActiveSubmaps(predicted_pose);
  result.local_pose = MatchToActiveSubmap(result.range_data, lidar_odom_pose);

  ++accumulated_scans_;
  result.valid = true;
  result.is_keyframe =
    accumulated_scans_ >= options_.scans_per_accumulation &&
    ShouldCreateKeyframe(result.local_pose);
  result.insertion_required = result.is_keyframe && options_.enable_map_update;

  if (result.insertion_required) {
    InsertIntoActiveSubmaps(result.range_data, result.local_pose);
    last_keyframe_pose_ = result.local_pose;
    has_last_keyframe_pose_ = true;
    accumulated_scans_ = 0;
  }

  if (has_pose_estimate_) {
    // 有外部里程计时，用局部位姿差分更新一个可观测的激光里程计轨迹。
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

const std::vector<std::shared_ptr<Submap2D>> & LocalSlamFrontend::active_submaps() const
{
  return active_submaps_;
}

const Pose2D & LocalSlamFrontend::lidar_odom_pose() const
{
  return lidar_odom_pose_estimate_;
}

RangeData2D LocalSlamFrontend::FilterScan(const sensor_msgs::msg::LaserScan & scan) const
{
  RangeData2D data;
  data.stamp = scan.header.stamp;

  double angle = scan.angle_min;
  for (const float range : scan.ranges) {
    if (std::isfinite(range) && range >= options_.min_range && range <= options_.max_range) {
      data.returns.push_back(Point2D{
        static_cast<double>(range) * std::cos(angle),
        static_cast<double>(range) * std::sin(angle)});
    }
    angle += scan.angle_increment;
  }

  return data;
}

RangeData2D LocalSlamFrontend::VoxelFilter(const RangeData2D & range_data) const
{
  if (options_.voxel_filter_size <= 0.0) {
    return range_data;
  }

  // 使用固定网格做最轻量的降采样，先把前端稳定跑起来。
  RangeData2D filtered;
  filtered.stamp = range_data.stamp;
  std::vector<Point2D> sorted_points = range_data.returns;
  std::sort(
    sorted_points.begin(), sorted_points.end(),
    [this](const Point2D & lhs, const Point2D & rhs) {
      const int lhs_x = static_cast<int>(std::floor(lhs.x / options_.voxel_filter_size));
      const int lhs_y = static_cast<int>(std::floor(lhs.y / options_.voxel_filter_size));
      const int rhs_x = static_cast<int>(std::floor(rhs.x / options_.voxel_filter_size));
      const int rhs_y = static_cast<int>(std::floor(rhs.y / options_.voxel_filter_size));
      if (lhs_x != rhs_x) {
        return lhs_x < rhs_x;
      }
      return lhs_y < rhs_y;
    });

  int last_x = std::numeric_limits<int>::min();
  int last_y = std::numeric_limits<int>::min();
  for (const auto & point : sorted_points) {
    const int cell_x = static_cast<int>(std::floor(point.x / options_.voxel_filter_size));
    const int cell_y = static_cast<int>(std::floor(point.y / options_.voxel_filter_size));
    if (cell_x == last_x && cell_y == last_y) {
      continue;
    }
    filtered.returns.push_back(point);
    last_x = cell_x;
    last_y = cell_y;
  }
  return filtered;
}

std::vector<Point2D> LocalSlamFrontend::DownsamplePoints(
  const std::vector<Point2D> & points,
  const int max_points) const
{
  if (max_points <= 0 || static_cast<int>(points.size()) <= max_points) {
    return points;
  }

  std::vector<Point2D> sampled_points;
  sampled_points.reserve(static_cast<size_t>(max_points));
  const size_t stride = std::max<size_t>(1, points.size() / static_cast<size_t>(max_points));
  for (size_t index = 0; index < points.size() && static_cast<int>(sampled_points.size()) < max_points;
    index += stride)
  {
    sampled_points.push_back(points[index]);
  }
  return sampled_points;
}

Pose2D LocalSlamFrontend::PoseFromOdom(const nav_msgs::msg::Odometry & odom_msg) const
{
  Pose2D odom_pose;
  odom_pose.x = odom_msg.pose.pose.position.x;
  odom_pose.y = odom_msg.pose.pose.position.y;

  tf2::Quaternion q;
  tf2::fromMsg(odom_msg.pose.pose.orientation, q);
  odom_pose.yaw = tf2::getYaw(q);
  return odom_pose;
}

Pose2D LocalSlamFrontend::PredictPose(const nav_msgs::msg::Odometry * odom_msg)
{
  // 预测项优先使用外部里程计；没有 /odom 时退回到激光里程计增量。
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
      previous_odom_pose_ = current_odom_pose;
      has_previous_odom_ = true;
      return local_pose_estimate_;
    }

    const Pose2D odom_delta = RelativePose(previous_odom_pose_, current_odom_pose);
    previous_odom_pose_ = current_odom_pose;
    return ComposePoses(local_pose_estimate_, odom_delta);
  }

  if (!has_previous_range_data_) {
    // 第一次没有 /odom 时先保持当前位姿，下一帧开始滚动激光里程计增量。
    return local_pose_estimate_;
  }

  return ComposePoses(local_pose_estimate_, lidar_odom_delta_);
}

Pose2D LocalSlamFrontend::MatchToPreviousScan(
  const RangeData2D & current_range_data,
  const Pose2D & initial_relative_pose) const
{
  if (!has_previous_range_data_ || previous_range_data_.returns.empty()) {
    return initial_relative_pose;
  }

  const auto current_points =
    DownsamplePoints(current_range_data.returns, options_.lidar_odom_max_points);
  const auto previous_points =
    DownsamplePoints(previous_range_data_.returns, options_.lidar_odom_max_points);
  if (current_points.empty() || previous_points.empty()) {
    return initial_relative_pose;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  source_cloud->reserve(current_points.size());
  target_cloud->reserve(previous_points.size());

  for (const auto & point : current_points) {
    source_cloud->push_back(pcl::PointXYZ(
      static_cast<float>(point.x),
      static_cast<float>(point.y),
      0.0F));
  }
  for (const auto & point : previous_points) {
    target_cloud->push_back(pcl::PointXYZ(
      static_cast<float>(point.x),
      static_cast<float>(point.y),
      0.0F));
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source_cloud);
  icp.setInputTarget(target_cloud);
  icp.setMaximumIterations(40);
  icp.setMaxCorrespondenceDistance(options_.lidar_odom_point_sigma);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACOutlierRejectionThreshold(options_.lidar_odom_point_sigma);

  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
  initial_guess(0, 0) = static_cast<float>(std::cos(initial_relative_pose.yaw));
  initial_guess(0, 1) = static_cast<float>(-std::sin(initial_relative_pose.yaw));
  initial_guess(1, 0) = static_cast<float>(std::sin(initial_relative_pose.yaw));
  initial_guess(1, 1) = static_cast<float>(std::cos(initial_relative_pose.yaw));
  initial_guess(0, 3) = static_cast<float>(initial_relative_pose.x);
  initial_guess(1, 3) = static_cast<float>(initial_relative_pose.y);

  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  icp.align(aligned_cloud, initial_guess);
  if (!icp.hasConverged()) {
    return initial_relative_pose;
  }

  const Eigen::Matrix4f transform = icp.getFinalTransformation();
  const double yaw = std::atan2(transform(1, 0), transform(0, 0));
  Pose2D relative_pose;
  relative_pose.x = static_cast<double>(transform(0, 3));
  relative_pose.y = static_cast<double>(transform(1, 3));
  relative_pose.yaw = NormalizeAngle(yaw);
  return relative_pose;
}

Pose2D LocalSlamFrontend::MatchToActiveSubmap(
  const RangeData2D & range_data,
  const Pose2D & predicted_pose) const
{
  if (active_submaps_.empty()) {
    return predicted_pose;
  }

  const auto & matching_submap = active_submaps_.back();
  if (!matching_submap->HasSufficientData()) {
    return predicted_pose;
  }

  // 这里先做实时相关匹配，保证前端不依赖后端也能独立收敛。
  Pose2D best_pose = predicted_pose;
  double best_score = ScoreCandidate(*matching_submap, range_data, predicted_pose, predicted_pose);

  for (double yaw_delta = -options_.scan_matcher.angular_window;
    yaw_delta <= options_.scan_matcher.angular_window + 1e-6;
    yaw_delta += options_.scan_matcher.angular_step)
  {
    for (double dx = -options_.scan_matcher.linear_window;
      dx <= options_.scan_matcher.linear_window + 1e-6;
      dx += options_.scan_matcher.linear_step)
    {
      for (double dy = -options_.scan_matcher.linear_window;
        dy <= options_.scan_matcher.linear_window + 1e-6;
        dy += options_.scan_matcher.linear_step)
      {
        Pose2D candidate = predicted_pose;
        candidate.x += dx;
        candidate.y += dy;
        candidate.yaw = NormalizeAngle(candidate.yaw + yaw_delta);

        const double score = ScoreCandidate(*matching_submap, range_data, candidate, predicted_pose);
        if (score > best_score) {
          best_score = score;
          best_pose = candidate;
        }
      }
    }
  }

  return best_pose;
}

double LocalSlamFrontend::ScoreCandidate(
  const Submap2D & submap,
  const RangeData2D & range_data,
  const Pose2D & candidate_pose,
  const Pose2D & predicted_pose) const
{
  double score = 0.0;
  for (const auto & hit_in_sensor : range_data.returns) {
    const Point2D hit_in_world = TransformPoint(hit_in_sensor, candidate_pose);
    score += submap.GetProbability(hit_in_world);
  }

  score /= static_cast<double>(range_data.returns.size());

  const double translation_penalty =
    std::hypot(candidate_pose.x - predicted_pose.x, candidate_pose.y - predicted_pose.y);
  const double rotation_penalty = std::abs(NormalizeAngle(candidate_pose.yaw - predicted_pose.yaw));

  return score
    - options_.scan_matcher.translation_delta_cost_weight * translation_penalty
    - options_.scan_matcher.rotation_delta_cost_weight * rotation_penalty;
}

bool LocalSlamFrontend::ShouldCreateKeyframe(const Pose2D & matched_pose) const
{
  if (!has_last_keyframe_pose_) {
    return true;
  }

  const double translation =
    std::hypot(matched_pose.x - last_keyframe_pose_.x, matched_pose.y - last_keyframe_pose_.y);
  const double rotation =
    std::abs(NormalizeAngle(matched_pose.yaw - last_keyframe_pose_.yaw));
  return translation >= options_.keyframe_translation_threshold ||
    rotation >= options_.keyframe_rotation_threshold;
}

void LocalSlamFrontend::InsertIntoActiveSubmaps(
  const RangeData2D & range_data,
  const Pose2D & matched_pose)
{
  // 当前版本使用两个重叠活动子图，先把 Cartographer 风格的数据流搭起来。
  MaybeGrowActiveSubmaps(matched_pose);
  for (auto & submap : active_submaps_) {
    if (!submap->IsFinished()) {
      submap->InsertRangeData(range_data, matched_pose);
    }
  }
  if (active_submaps_.size() == 2 && active_submaps_.front()->IsFinished()) {
    active_submaps_.erase(active_submaps_.begin());
  }
}

void LocalSlamFrontend::MaybeGrowActiveSubmaps(const Pose2D & matched_pose)
{
  if (active_submaps_.empty()) {
    active_submaps_.push_back(
      std::make_shared<Submap2D>(next_submap_id_++, options_.submap, matched_pose));
    return;
  }

  if (!options_.enable_map_update) {
    return;
  }

  if (active_submaps_.size() == 1 &&
    active_submaps_.back()->num_insertions() >= options_.active_submap_num_range_data / 2)
  {
    active_submaps_.push_back(
      std::make_shared<Submap2D>(next_submap_id_++, options_.submap, matched_pose));
  }
}

}  // namespace simple_slam

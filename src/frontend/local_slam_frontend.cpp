#include "simple_slam/frontend/local_slam_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
  MaybeGrowActiveSubmaps(predicted_pose);
  result.local_pose = MatchToActiveSubmap(result.range_data, predicted_pose);

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

  local_pose_estimate_ = result.local_pose;
  has_pose_estimate_ = true;
  return result;
}

const std::vector<std::shared_ptr<Submap2D>> & LocalSlamFrontend::active_submaps() const
{
  return active_submaps_;
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

Pose2D LocalSlamFrontend::PredictPose(const nav_msgs::msg::Odometry * odom_msg)
{
  // 里程计只作为预测项，不直接当作最终位姿。
  if (!has_pose_estimate_) {
    if (odom_msg != nullptr) {
      previous_odom_pose_ = Pose2D{
        odom_msg->pose.pose.position.x,
        odom_msg->pose.pose.position.y,
        0.0};
      tf2::Quaternion q;
      tf2::fromMsg(odom_msg->pose.pose.orientation, q);
      previous_odom_pose_.yaw = tf2::getYaw(q);
      has_previous_odom_ = true;
      return previous_odom_pose_;
    }
    return Pose2D{};
  }

  if (odom_msg == nullptr) {
    return local_pose_estimate_;
  }

  Pose2D current_odom_pose;
  current_odom_pose.x = odom_msg->pose.pose.position.x;
  current_odom_pose.y = odom_msg->pose.pose.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(odom_msg->pose.pose.orientation, q);
  current_odom_pose.yaw = tf2::getYaw(q);

  if (!has_previous_odom_) {
    previous_odom_pose_ = current_odom_pose;
    has_previous_odom_ = true;
    return local_pose_estimate_;
  }

  const Pose2D odom_delta = RelativePose(previous_odom_pose_, current_odom_pose);
  previous_odom_pose_ = current_odom_pose;
  return ComposePoses(local_pose_estimate_, odom_delta);
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

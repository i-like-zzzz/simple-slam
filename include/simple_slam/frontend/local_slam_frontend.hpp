#ifndef SIMPLE_SLAM__FRONTEND__LOCAL_SLAM_FRONTEND_HPP_
#define SIMPLE_SLAM__FRONTEND__LOCAL_SLAM_FRONTEND_HPP_

#include <memory>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "simple_slam/mapping/submap_2d.hpp"
#include "simple_slam/types.hpp"

namespace simple_slam
{

class LocalSlamFrontend
{
public:
  struct Options
  {
    double min_range = 0.05;
    double max_range = 20.0;
    double voxel_filter_size = 0.05;
    int scans_per_accumulation = 1;
    int active_submap_num_range_data = 90;
    int min_range_points_for_match = 20;
    bool enable_map_update = true;
    double keyframe_translation_threshold = 0.2;
    double keyframe_rotation_threshold = 0.17;
    double lidar_odom_linear_window = 0.2;
    double lidar_odom_angular_window = 0.2;
    double lidar_odom_translation_weight = 1.0;
    double lidar_odom_rotation_weight = 0.2;
    double lidar_odom_point_sigma = 0.15;
    int lidar_odom_max_points = 48;
    SearchParameters2D scan_matcher;
    Submap2D::Options submap;
  };

  explicit LocalSlamFrontend(Options options);

  LocalSlamResult2D AddScan(
    const sensor_msgs::msg::LaserScan & scan,
    const nav_msgs::msg::Odometry * odom_msg);
  const std::vector<std::shared_ptr<Submap2D>> & active_submaps() const;
  const Pose2D & lidar_odom_pose() const;

private:
  RangeData2D FilterScan(const sensor_msgs::msg::LaserScan & scan) const;
  RangeData2D VoxelFilter(const RangeData2D & range_data) const;
  std::vector<Point2D> DownsamplePoints(const std::vector<Point2D> & points, int max_points) const;
  Pose2D PoseFromOdom(const nav_msgs::msg::Odometry & odom_msg) const;
  Pose2D PredictPose(const nav_msgs::msg::Odometry * odom_msg);
  Pose2D MatchToPreviousScan(
    const RangeData2D & current_range_data,
    const Pose2D & initial_relative_pose) const;
  Pose2D MatchToActiveSubmap(const RangeData2D & range_data, const Pose2D & predicted_pose) const;
  double ScoreCandidate(
    const Submap2D & submap,
    const RangeData2D & range_data,
    const Pose2D & candidate_pose,
    const Pose2D & predicted_pose) const;
  bool ShouldCreateKeyframe(const Pose2D & matched_pose) const;
  void InsertIntoActiveSubmaps(const RangeData2D & range_data, const Pose2D & matched_pose);
  void MaybeGrowActiveSubmaps(const Pose2D & matched_pose);

  Options options_;
  int accumulated_scans_ = 0;
  bool has_pose_estimate_ = false;
  bool has_previous_odom_ = false;
  bool has_last_keyframe_pose_ = false;
  bool has_previous_range_data_ = false;
  Pose2D local_pose_estimate_;
  Pose2D lidar_odom_pose_estimate_;
  Pose2D previous_odom_pose_;
  Pose2D last_keyframe_pose_;
  Pose2D lidar_odom_delta_;
  RangeData2D previous_range_data_;
  int next_submap_id_ = 0;
  std::vector<std::shared_ptr<Submap2D>> active_submaps_;
};

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__FRONTEND__LOCAL_SLAM_FRONTEND_HPP_

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

// 局部前端负责 scan 预处理、激光里程计和 scan-to-submap 对齐。
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

  // 前端主入口：输入一帧 LaserScan，输出局部位姿和关键帧状态。
  LocalSlamResult2D AddScan(
    const sensor_msgs::msg::LaserScan & scan,
    const nav_msgs::msg::Odometry * odom_msg);

  // 当前仍在参与匹配/插入的活动子图。
  const std::vector<std::shared_ptr<Submap2D>> & active_submaps() const;

  // 单独暴露激光里程计轨迹，便于和最终 local_pose 对比。
  const Pose2D & lidar_odom_pose() const;

private:
  // LaserScan -> 点云，并过滤无效量测。
  RangeData2D FilterScan(const sensor_msgs::msg::LaserScan & scan) const;

  // 简单体素滤波，降低前端匹配代价。
  RangeData2D VoxelFilter(const RangeData2D & range_data) const;

  // 控制 ICP 输入点数上限，避免 bag 回放时计算量过大。
  std::vector<Point2D> DownsamplePoints(const std::vector<Point2D> & points, int max_points) const;

  // ROS 里程计消息转内部 Pose2D。
  Pose2D PoseFromOdom(const nav_msgs::msg::Odometry & odom_msg) const;

  // 预测下一帧位姿；优先用外部里程计，其次用激光里程计增量。
  Pose2D PredictPose(const nav_msgs::msg::Odometry * odom_msg);

  // 基于相邻两帧点云做 ICP，估计相对运动。
  Pose2D MatchToPreviousScan(
    const RangeData2D & current_range_data,
    const Pose2D & initial_relative_pose) const;

  // 基于活动子图做 scan-to-submap 粗匹配。
  Pose2D MatchToActiveSubmap(const RangeData2D & range_data, const Pose2D & predicted_pose) const;

  // 计算某个候选位姿在当前活动子图上的匹配分数。
  double ScoreCandidate(
    const Submap2D & submap,
    const RangeData2D & range_data,
    const Pose2D & candidate_pose,
    const Pose2D & predicted_pose) const;

  // 根据相邻关键帧位姿差判断是否需要插入新关键帧。
  bool ShouldCreateKeyframe(const Pose2D & matched_pose) const;

  // 把关键帧插入所有活动子图。
  void InsertIntoActiveSubmaps(const RangeData2D & range_data, const Pose2D & matched_pose);

  // 维护两个重叠活动子图的生命周期。
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

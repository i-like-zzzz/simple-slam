#ifndef SIMPLE_SLAM__SIMPLE_SLAM_NODE_HPP_
#define SIMPLE_SLAM__SIMPLE_SLAM_NODE_HPP_

#include <memory>
#include <optional>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "simple_slam/backend/pose_graph_backend.hpp"
#include "simple_slam/frontend/local_slam_frontend.hpp"
#include "simple_slam/optimization/pose_graph_2d.hpp"
#include "simple_slam/system/system_mode.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace simple_slam
{

// ROS 2 包装节点：负责订阅传感器、驱动前端，并发布调试结果。
class SimpleSlamNode : public rclcpp::Node
{
public:
  SimpleSlamNode();

private:
  // 处理一帧激光，并驱动一次局部 SLAM 更新。
  void HandleScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // 缓存最新外部里程计，供前端做预测。
  void HandleOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  // 统一发布路径、激光里程计和 TF。
  void PublishOutputs(const LocalSlamResult2D & result, const std::string & scan_frame);

  // 发布关键帧箭头，方便在 RViz 里观察插帧位置。
  void PublishKeyframeMarkers(const Pose2D & pose, const rclcpp::Time & stamp);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr laser_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframe_marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  std::optional<Pose2D> smoothed_tf_pose_;
  nav_msgs::msg::Path path_msg_;
  std::vector<Pose2D> keyframe_poses_;
  int processed_scan_count_ = 0;

  std::unique_ptr<LocalSlamFrontend> frontend_;
  std::unique_ptr<PoseGraph2D> pose_graph_;
  std::unique_ptr<PoseGraphBackend> backend_;

  SystemMode system_mode_ = SystemMode::kMapping;
  bool publish_keyframe_markers_ = true;
  bool enable_tf_smoothing_ = false;
  double keyframe_marker_scale_ = 0.25;
  double tf_translation_alpha_ = 1.0;
  double tf_rotation_alpha_ = 1.0;
  int debug_log_every_n_scans_ = 100;

  std::string map_frame_;
  std::string odom_frame_;
  std::string published_frame_;
};

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__SIMPLE_SLAM_NODE_HPP_

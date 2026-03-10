#ifndef SIMPLE_SLAM__SIMPLE_SLAM_NODE_HPP_
#define SIMPLE_SLAM__SIMPLE_SLAM_NODE_HPP_

#include <memory>
#include <optional>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "simple_slam/backend/pose_graph_backend.hpp"
#include "simple_slam/frontend/local_slam_frontend.hpp"
#include "simple_slam/optimization/pose_graph_2d.hpp"
#include "simple_slam/system/system_mode.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace simple_slam
{

class SimpleSlamNode : public rclcpp::Node
{
public:
  SimpleSlamNode();

private:
  void HandleScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void HandleOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void PublishOutputs(const LocalSlamResult2D & result);
  void PublishKeyframeMarkers(const LocalSlamResult2D & result, const rclcpp::Time & stamp);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr laser_odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframe_marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  nav_msgs::msg::Path path_msg_;
  std::vector<Pose2D> keyframe_poses_;
  int processed_scan_count_ = 0;

  std::unique_ptr<LocalSlamFrontend> frontend_;
  std::unique_ptr<PoseGraph2D> pose_graph_;
  std::unique_ptr<PoseGraphBackend> backend_;

  SystemMode system_mode_ = SystemMode::kMapping;
  bool publish_keyframe_markers_ = true;
  double keyframe_marker_scale_ = 0.25;
  int debug_log_every_n_scans_ = 100;

  std::string map_frame_;
  std::string odom_frame_;
  std::string published_frame_;
};

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__SIMPLE_SLAM_NODE_HPP_

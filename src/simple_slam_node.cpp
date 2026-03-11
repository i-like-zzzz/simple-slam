#include "simple_slam/simple_slam_node.hpp"

#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_ros/create_timer_ros.h"
#include "visualization_msgs/msg/marker.hpp"

namespace simple_slam
{

namespace
{

Pose2D PoseFromOdometry(const nav_msgs::msg::Odometry & odom_msg)
{
  Pose2D pose;
  pose.x = odom_msg.pose.pose.position.x;
  pose.y = odom_msg.pose.pose.position.y;
  pose.yaw = tf2::getYaw(odom_msg.pose.pose.orientation);
  return pose;
}

Pose2D PoseFromTransform(const geometry_msgs::msg::Transform & transform)
{
  Pose2D pose;
  pose.x = transform.translation.x;
  pose.y = transform.translation.y;
  pose.yaw = tf2::getYaw(transform.rotation);
  return pose;
}

}  // namespace

SimpleSlamNode::SimpleSlamNode()
: Node("simple_slam_node")
{
  // 这些参数放在节点入口统一声明，后面切换建图/定位模式时不会拆散接口。
  map_frame_ = declare_parameter("map_frame", "map");
  odom_frame_ = declare_parameter("odom_frame", "odom");
  published_frame_ = declare_parameter("published_frame", "base_link");
  system_mode_ = ParseSystemMode(declare_parameter("system_mode", std::string("mapping")));
  publish_keyframe_markers_ = declare_parameter("publish_keyframe_markers", true);
  enable_tf_smoothing_ = declare_parameter("enable_tf_smoothing", false);
  keyframe_marker_scale_ = declare_parameter("keyframe_marker_scale", 0.25);
  tf_translation_alpha_ = declare_parameter("tf_translation_alpha", 1.0);
  tf_rotation_alpha_ = declare_parameter("tf_rotation_alpha", 1.0);
  debug_log_every_n_scans_ = static_cast<int>(declare_parameter("debug_log_every_n_scans", 100));
  const bool enable_backend = declare_parameter("enable_backend", false);

  const auto active_submap_num_range_data =
    static_cast<int>(declare_parameter("active_submap_num_range_data", 90));
  const bool enable_map_update = system_mode_ == SystemMode::kMapping;

  frontend_ = std::make_unique<LocalSlamFrontend>(LocalSlamFrontend::Options{
      declare_parameter("min_range", 0.05),
      declare_parameter("max_range", 20.0),
      declare_parameter("voxel_filter_size", 0.05),
      static_cast<int>(declare_parameter("scans_per_accumulation", 1)),
      active_submap_num_range_data,
      static_cast<int>(declare_parameter("min_range_points_for_match", 20)),
      enable_map_update,
      declare_parameter("keyframe_translation_threshold", 0.2),
      declare_parameter("keyframe_rotation_threshold", 0.17),
      declare_parameter("lidar_odom_linear_window", 0.2),
      declare_parameter("lidar_odom_angular_window", 0.2),
      declare_parameter("lidar_odom_translation_weight", 1.0),
      declare_parameter("lidar_odom_rotation_weight", 0.2),
      declare_parameter("lidar_odom_point_sigma", 0.15),
      static_cast<int>(declare_parameter("lidar_odom_max_points", 48)),
      SearchParameters2D{
        declare_parameter("linear_search_window", 0.3),
        declare_parameter("angular_search_window", 0.35),
        declare_parameter("linear_search_step", 0.05),
        declare_parameter("angular_search_step", 0.05),
        declare_parameter("translation_delta_cost_weight", 1.0),
        declare_parameter("rotation_delta_cost_weight", 0.2)},
      Submap2D::Options{
        declare_parameter("submap_resolution", 0.05),
        static_cast<int>(declare_parameter("submap_width", 400)),
        static_cast<int>(declare_parameter("submap_height", 400)),
        active_submap_num_range_data,
        declare_parameter("submap_hit_probability", 0.7),
        declare_parameter("submap_miss_probability", 0.49)}});
  pose_graph_ = std::make_unique<PoseGraph2D>(PoseGraph2D::Options{
      active_submap_num_range_data});
  backend_ = std::make_unique<PoseGraphBackend>(PoseGraphBackend::Options{enable_backend});

  path_msg_.header.frame_id = map_frame_;

  path_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory", 10);
  // 单独发布激光里程计，便于和最终局部位姿做对比。
  laser_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("laser_odom", 10);
  current_scan_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("current_scan_cloud", 10);
  keyframe_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("keyframes", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 传感器话题统一用 SensorDataQoS，兼容仿真和 rosbag 回放里的 best_effort 发布者。
  const auto sensor_qos = rclcpp::SensorDataQoS();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", sensor_qos, std::bind(&SimpleSlamNode::HandleOdom, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", sensor_qos, std::bind(&SimpleSlamNode::HandleScan, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "simple_slam_node started, mode=%s, backend=%s, keyframe_markers=%s, published_frame=%s",
    ToString(system_mode_),
    backend_->enabled() ? "on" : "off",
    publish_keyframe_markers_ ? "on" : "off",
    published_frame_.c_str());
}

void SimpleSlamNode::HandleScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const nav_msgs::msg::Odometry * odom_ptr = latest_odom_ ? &(*latest_odom_) : nullptr;
  auto result = frontend_->AddScan(*msg, odom_ptr);
  if (!result.valid) {
    return;
  }

  pose_graph_->AddNode(result);
  pose_graph_->RegisterSubmaps(frontend_->active_submaps());
  backend_->AddLocalSlamResult(result);
  ++processed_scan_count_;
  if (debug_log_every_n_scans_ > 0 && processed_scan_count_ % debug_log_every_n_scans_ == 0) {
    RCLCPP_INFO(
      get_logger(),
      "frontend status: scans=%d pose=(%.3f, %.3f, %.3f) keyframes=%zu insertion=%s",
      processed_scan_count_,
      result.local_pose.x,
      result.local_pose.y,
      result.local_pose.yaw,
      keyframe_poses_.size(),
      result.insertion_required ? "true" : "false");
  }
  PublishOutputs(result, msg->header.frame_id);
}

void SimpleSlamNode::HandleOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
}

void SimpleSlamNode::PublishOutputs(
  const LocalSlamResult2D & result,
  const std::string & scan_frame)
{
  Pose2D published_pose = result.local_pose;
  Pose2D published_to_scan;
  if (!scan_frame.empty() && scan_frame != published_frame_) {
    try {
      const auto published_to_scan_tf =
        tf_buffer_->lookupTransform(published_frame_, scan_frame, tf2::TimePointZero);
      published_to_scan = PoseFromTransform(published_to_scan_tf.transform);
      published_pose = ComposePoses(result.local_pose, InversePose(published_to_scan));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "failed to lookup transform %s -> %s: %s",
        published_frame_.c_str(), scan_frame.c_str(), ex.what());
    }
  }

  Pose2D laser_odom_pose = frontend_->lidar_odom_pose();
  if (!scan_frame.empty() && scan_frame != published_frame_) {
    laser_odom_pose = ComposePoses(frontend_->lidar_odom_pose(), InversePose(published_to_scan));
  }

  // 路径和 TF 先保持稳定输出，方便之后单独观察前端漂移。
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = map_frame_;
  pose_stamped.header.stamp = rclcpp::Time(result.range_data.stamp);
  pose_stamped.pose = ToRosPose(published_pose);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, published_pose.yaw);
  pose_stamped.pose.orientation = tf2::toMsg(q);

  path_msg_.header.stamp = pose_stamped.header.stamp;
  path_msg_.poses.push_back(pose_stamped);
  path_pub_->publish(path_msg_);

  nav_msgs::msg::Odometry laser_odom_msg;
  laser_odom_msg.header.stamp = pose_stamped.header.stamp;
  laser_odom_msg.header.frame_id = map_frame_;
  laser_odom_msg.child_frame_id = published_frame_;
  // 这里发的是纯前端 ICP 累积结果，不是经过子图匹配修正后的最终轨迹。
  laser_odom_msg.pose.pose = ToRosPose(laser_odom_pose);
  tf2::Quaternion laser_odom_q;
  laser_odom_q.setRPY(0.0, 0.0, laser_odom_pose.yaw);
  laser_odom_msg.pose.pose.orientation = tf2::toMsg(laser_odom_q);
  laser_odom_pub_->publish(laser_odom_msg);

  sensor_msgs::msg::PointCloud2 current_scan_cloud;
  current_scan_cloud.header.frame_id = map_frame_;
  current_scan_cloud.header.stamp = pose_stamped.header.stamp;
  current_scan_cloud.height = 1;
  current_scan_cloud.width = static_cast<uint32_t>(result.range_data.returns.size());

  sensor_msgs::PointCloud2Modifier modifier(current_scan_cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(result.range_data.returns.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(current_scan_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(current_scan_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(current_scan_cloud, "z");
  for (const auto & point_in_scan : result.range_data.returns) {
    const auto point_in_published = TransformPoint(point_in_scan, published_to_scan);
    const auto point_in_map = TransformPoint(point_in_published, published_pose);
    *iter_x = static_cast<float>(point_in_map.x);
    *iter_y = static_cast<float>(point_in_map.y);
    *iter_z = 0.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  current_scan_cloud_pub_->publish(current_scan_cloud);

  Pose2D map_to_odom_pose = published_pose;
  if (
    latest_odom_ &&
    latest_odom_->header.frame_id == odom_frame_ &&
    latest_odom_->child_frame_id == published_frame_)
  {
    map_to_odom_pose = ComposePoses(published_pose, InversePose(PoseFromOdometry(*latest_odom_)));
  } else if (odom_frame_ != published_frame_) {
    try {
      const auto odom_to_published_tf =
        tf_buffer_->lookupTransform(odom_frame_, published_frame_, tf2::TimePointZero);
      map_to_odom_pose = ComposePoses(
        published_pose, InversePose(PoseFromTransform(odom_to_published_tf.transform)));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "failed to lookup transform %s -> %s: %s",
        odom_frame_.c_str(), published_frame_.c_str(), ex.what());
    }
  }

  Pose2D tf_pose_to_publish = map_to_odom_pose;
  if (enable_tf_smoothing_) {
    if (!smoothed_tf_pose_) {
      smoothed_tf_pose_ = map_to_odom_pose;
    } else {
      smoothed_tf_pose_->x += tf_translation_alpha_ * (map_to_odom_pose.x - smoothed_tf_pose_->x);
      smoothed_tf_pose_->y += tf_translation_alpha_ * (map_to_odom_pose.y - smoothed_tf_pose_->y);
      const double yaw_delta = NormalizeAngle(map_to_odom_pose.yaw - smoothed_tf_pose_->yaw);
      smoothed_tf_pose_->yaw = NormalizeAngle(
        smoothed_tf_pose_->yaw + tf_rotation_alpha_ * yaw_delta);
    }
    tf_pose_to_publish = *smoothed_tf_pose_;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = pose_stamped.header.stamp;
  tf_msg.header.frame_id = map_frame_;
  tf_msg.child_frame_id = odom_frame_;
  tf_msg.transform.translation.x = tf_pose_to_publish.x;
  tf_msg.transform.translation.y = tf_pose_to_publish.y;
  tf_msg.transform.translation.z = 0.0;
  tf2::Quaternion map_to_odom_q;
  map_to_odom_q.setRPY(0.0, 0.0, tf_pose_to_publish.yaw);
  tf_msg.transform.rotation = tf2::toMsg(map_to_odom_q);
  tf_broadcaster_->sendTransform(tf_msg);

  if (result.is_keyframe) {
    PublishKeyframeMarkers(published_pose, pose_stamped.header.stamp);
  }
}

void SimpleSlamNode::PublishKeyframeMarkers(
  const Pose2D & pose,
  const rclcpp::Time & stamp)
{
  if (!publish_keyframe_markers_) {
    return;
  }

  keyframe_poses_.push_back(pose);

  visualization_msgs::msg::MarkerArray marker_array;
  for (size_t index = 0; index < keyframe_poses_.size(); ++index) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = stamp;
    marker.ns = "simple_slam_keyframes";
    marker.id = static_cast<int>(index);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = ToRosPose(keyframe_poses_[index]);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, keyframe_poses_[index].yaw);
    marker.pose.orientation = tf2::toMsg(q);

    marker.scale.x = keyframe_marker_scale_;
    marker.scale.y = keyframe_marker_scale_ * 0.3;
    marker.scale.z = keyframe_marker_scale_ * 0.3;
    marker.color.a = 1.0;

    if (index + 1 == keyframe_poses_.size()) {
      // 当前关键帧用更亮的颜色单独强调。
      marker.color.r = 1.0;
      marker.color.g = 0.2;
      marker.color.b = 0.2;
    } else {
      marker.color.r = 0.1;
      marker.color.g = 0.8;
      marker.color.b = 0.2;
    }
    marker_array.markers.push_back(marker);
  }

  keyframe_marker_pub_->publish(marker_array);
}

}  // namespace simple_slam

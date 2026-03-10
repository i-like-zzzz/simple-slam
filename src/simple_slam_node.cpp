#include "simple_slam/simple_slam_node.hpp"

#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace simple_slam
{

SimpleSlamNode::SimpleSlamNode()
: Node("simple_slam_node")
{
  // 这些参数放在节点入口统一声明，后面切换建图/定位模式时不会拆散接口。
  map_frame_ = declare_parameter("map_frame", "map");
  odom_frame_ = declare_parameter("odom_frame", "odom");
  published_frame_ = declare_parameter("published_frame", "base_link");
  system_mode_ = ParseSystemMode(declare_parameter("system_mode", std::string("mapping")));
  publish_keyframe_markers_ = declare_parameter("publish_keyframe_markers", true);
  keyframe_marker_scale_ = declare_parameter("keyframe_marker_scale", 0.25);
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
  keyframe_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("keyframes", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", 50, std::bind(&SimpleSlamNode::HandleOdom, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 50, std::bind(&SimpleSlamNode::HandleScan, this, std::placeholders::_1));

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
  PublishOutputs(result);
}

void SimpleSlamNode::HandleOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
}

void SimpleSlamNode::PublishOutputs(const LocalSlamResult2D & result)
{
  // 路径和 TF 先保持稳定输出，方便之后单独观察前端漂移。
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = map_frame_;
  pose_stamped.header.stamp = rclcpp::Time(result.range_data.stamp);
  pose_stamped.pose = ToRosPose(result.local_pose);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, result.local_pose.yaw);
  pose_stamped.pose.orientation = tf2::toMsg(q);

  path_msg_.header.stamp = pose_stamped.header.stamp;
  path_msg_.poses.push_back(pose_stamped);
  path_pub_->publish(path_msg_);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = pose_stamped.header.stamp;
  tf_msg.header.frame_id = map_frame_;
  tf_msg.child_frame_id = odom_frame_;
  tf_msg.transform.translation.x = 0.0;
  tf_msg.transform.translation.y = 0.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(tf_msg);

  if (result.is_keyframe) {
    PublishKeyframeMarkers(result, pose_stamped.header.stamp);
  }
}

void SimpleSlamNode::PublishKeyframeMarkers(
  const LocalSlamResult2D & result,
  const rclcpp::Time & stamp)
{
  if (!publish_keyframe_markers_) {
    return;
  }

  keyframe_poses_.push_back(result.local_pose);

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

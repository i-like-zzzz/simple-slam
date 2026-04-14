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

#include "simple_slam/simple_slam_node.hpp"

#include <string>
#include <utility>
#include <algorithm>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "visualization_msgs/msg/marker.hpp"

namespace simple_slam {

namespace {

Pose2D PoseFromOdometry(const nav_msgs::msg::Odometry& odom_msg) {
  Pose2D pose;
  pose.x = odom_msg.pose.pose.position.x;
  pose.y = odom_msg.pose.pose.position.y;
  pose.yaw = tf2::getYaw(odom_msg.pose.pose.orientation);
  return pose;
}

Pose2D PoseFromTransform(const geometry_msgs::msg::Transform& transform) {
  Pose2D pose;
  pose.x = transform.translation.x;
  pose.y = transform.translation.y;
  pose.yaw = tf2::getYaw(transform.rotation);
  return pose;
}

LidarOdomMatcherType ParseLidarOdomMatcherType(const std::string& matcher_name,
                                               bool* recognized = nullptr) {
  if (recognized != nullptr) {
    *recognized = true;
  }
  if (matcher_name == "point_to_point_icp") {
    return LidarOdomMatcherType::kPointToPointIcp;
  }
  if (matcher_name == "generalized_icp") {
    return LidarOdomMatcherType::kGeneralizedIcp;
  }
  if (matcher_name == "correlative") {
    return LidarOdomMatcherType::kCorrelative;
  }
  if (matcher_name == "ceres") {
    return LidarOdomMatcherType::kCeres;
  }
  if (recognized != nullptr) {
    *recognized = false;
  }
  return LidarOdomMatcherType::kPointToPointIcp;
}

const char* ToString(LidarOdomMatcherType matcher_type) {
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

SimpleSlamNode::SimpleSlamNode() : Node("simple_slam_node") {
  // 这些参数放在节点入口统一声明，后面切换建图/定位模式时不会拆散接口。
  map_frame_ = declare_parameter("map_frame", "map");
  odom_frame_ = declare_parameter("odom_frame", "odom");
  published_frame_ = declare_parameter("published_frame", "base_link");
  system_mode_ =
      ParseSystemMode(declare_parameter("system_mode", std::string("mapping")));
  publish_keyframe_markers_ =
      declare_parameter("publish_keyframe_markers", true);
  enable_tf_smoothing_ = declare_parameter("enable_tf_smoothing", false);
  keyframe_marker_scale_ = declare_parameter("keyframe_marker_scale", 0.25);
  tf_translation_alpha_ = declare_parameter("tf_translation_alpha", 1.0);
  tf_rotation_alpha_ = declare_parameter("tf_rotation_alpha", 1.0);
  debug_log_every_n_scans_ =
      static_cast<int>(declare_parameter("debug_log_every_n_scans", 100));
  const bool enable_backend = declare_parameter("enable_backend", false);

  const auto active_submap_num_range_data =
      static_cast<int>(declare_parameter("active_submap_num_range_data", 90));
  const bool enable_map_update = system_mode_ == SystemMode::kMapping;
  const auto lidar_odom_matcher_name = declare_parameter(
      "lidar_odom_matcher", std::string("point_to_point_icp"));
  bool lidar_odom_matcher_recognized = false;
  const auto lidar_odom_matcher = ParseLidarOdomMatcherType(
      lidar_odom_matcher_name, &lidar_odom_matcher_recognized);
  if (!lidar_odom_matcher_recognized) {
    RCLCPP_WARN(
        get_logger(),
        "unknown lidar_odom_matcher '%s', falling back to point_to_point_icp",
        lidar_odom_matcher_name.c_str());
  }

  LocalSlamFrontend::Options frontend_options;
  frontend_options.min_range = declare_parameter("min_range", 0.05);
  frontend_options.max_range = declare_parameter("max_range", 20.0);
  frontend_options.voxel_filter_size =
      declare_parameter("voxel_filter_size", 0.05);
  frontend_options.scans_per_accumulation =
      static_cast<int>(declare_parameter("scans_per_accumulation", 1));
  frontend_options.active_submap_num_range_data = active_submap_num_range_data;
  frontend_options.min_range_points_for_match =
      static_cast<int>(declare_parameter("min_range_points_for_match", 20));
  frontend_options.enable_map_update = enable_map_update;
  frontend_options.keyframe_translation_threshold =
      declare_parameter("keyframe_translation_threshold", 0.2);
  frontend_options.keyframe_rotation_threshold =
      declare_parameter("keyframe_rotation_threshold", 0.17);
  frontend_options.lidar_odom_linear_window =
      declare_parameter("lidar_odom_linear_window", 0.2);
  frontend_options.lidar_odom_angular_window =
      declare_parameter("lidar_odom_angular_window", 0.2);
  frontend_options.lidar_odom_translation_weight =
      declare_parameter("lidar_odom_translation_weight", 1.0);
  frontend_options.lidar_odom_rotation_weight =
      declare_parameter("lidar_odom_rotation_weight", 0.2);
  frontend_options.lidar_odom_point_sigma =
      declare_parameter("lidar_odom_point_sigma", 0.15);
  frontend_options.lidar_odom_max_points =
      static_cast<int>(declare_parameter("lidar_odom_max_points", 48));
  frontend_options.lidar_odom_max_iterations =
      static_cast<int>(declare_parameter("lidar_odom_max_iterations", 40));
  frontend_options.lidar_odom_matcher = lidar_odom_matcher;
  frontend_options.scan_matcher = SearchParameters2D{
      declare_parameter("linear_search_window", 0.3),
      declare_parameter("angular_search_window", 0.35),
      declare_parameter("linear_search_step", 0.05),
      declare_parameter("angular_search_step", 0.05),
      declare_parameter("translation_delta_cost_weight", 1.0),
      declare_parameter("rotation_delta_cost_weight", 0.2)};
  frontend_options.submap = Submap2D::Options{
      declare_parameter("submap_resolution", 0.05),
      static_cast<int>(declare_parameter("submap_width", 400)),
      static_cast<int>(declare_parameter("submap_height", 400)),
      active_submap_num_range_data,
      declare_parameter("submap_hit_probability", 0.7),
      declare_parameter("submap_miss_probability", 0.49)};
  frontend_options.lidar_odom_ceres_max_correspondence_distance =
      declare_parameter("lidar_odom_ceres_max_correspondence_distance", 0.3);
  frontend_options.lidar_odom_ceres_huber_scale =
      declare_parameter("lidar_odom_ceres_huber_scale", 0.1);
  frontend_options.lidar_odom_ceres_max_num_iterations = static_cast<int>(
      declare_parameter("lidar_odom_ceres_max_num_iterations", 20));
  frontend_ = std::make_unique<LocalSlamFrontend>(frontend_options);
  pose_graph_ = std::make_unique<PoseGraph2D>(
      PoseGraph2D::Options{active_submap_num_range_data});
  backend_ = std::make_unique<PoseGraphBackend>(
      PoseGraphBackend::Options{enable_backend});

  path_msg_.header.frame_id = map_frame_;

  path_pub_ = create_publisher<nav_msgs::msg::Path>("trajectory", 10);
  // 单独发布激光里程计，便于和最终局部位姿做对比。
  laser_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("laser_odom", 10);
  current_scan_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("current_scan_cloud", 10);
  keyframe_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("keyframes", 10);
  active_submap_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("active_submap", 10);
  local_map_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("local_map", 10);
  global_map_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("global_map", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 传感器话题统一用 SensorDataQoS，兼容仿真和 rosbag 回放里的 best_effort
  // 发布者。
  const auto sensor_qos = rclcpp::SensorDataQoS();
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", sensor_qos,
      std::bind(&SimpleSlamNode::HandleOdom, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", sensor_qos,
      std::bind(&SimpleSlamNode::HandleScan, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
              "simple_slam_node started, mode=%s, backend=%s, "
              "keyframe_markers=%s, published_frame=%s, "
              "lidar_odom_matcher=%s",
              ToString(system_mode_), backend_->enabled() ? "on" : "off",
              publish_keyframe_markers_ ? "on" : "off",
              published_frame_.c_str(), ToString(lidar_odom_matcher));
}

void SimpleSlamNode::HandleScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // TODO(zwc): 时间戳没有保证同步。
  const nav_msgs::msg::Odometry* odom_ptr =
      latest_odom_ ? &(*latest_odom_) : nullptr;
  // 前端处理得到位姿。
  auto result = frontend_->AddScan(*msg, odom_ptr);
  if (!result.valid) {
    return;
  }

  // 每一帧有效 scan 都先登记成 node；是否参与约束和优化，取决于后面
  // 是否成为关键帧并真正插入活动子图。
  const int node_id = pose_graph_->AddNode(result);
  pose_graph_->RegisterSubmaps(frontend_->GetActiveSubmaps());

  // 关键帧一旦插入到活动子图，就同步记录一条 node-submap 约束。
  // 当前记录的是：
  //   relative_pose = T_submap_node
  // 后端后面会使用：
  //   T_map_node ~= T_map_submap * T_submap_node
  // 来统一优化节点和子图的全局位姿。
  if (result.insertion_required && node_id >= 0) {
    const auto& active_submaps = frontend_->GetActiveSubmaps();
    for (const int submap_id : result.insertion_submap_ids) {
      const auto submap_it = std::find_if(
          active_submaps.begin(), active_submaps.end(),
          [submap_id](const std::shared_ptr<Submap2D>& submap) {
            return submap->id() == submap_id;
          });

      if (submap_it == active_submaps.end()) {
        continue;
      }
      Constraint2D constraint;
      constraint.node_id = node_id;
      constraint.submap_id = submap_id;

      // submap->global_pose() 是 T_map_submap，result.local_pose 是 T_map_node，
      // 所以 RelativePose(submap, node) 得到的正是约束里最核心的 T_submap_node。
      constraint.relative_pose =
          RelativePose((*submap_it)->global_pose(), result.local_pose);
      constraint.translation_weight = 1.0;
      constraint.rotation_weight = 1.0;
      constraint.tag = ConstraintTag::kIntraSubmap;
      pose_graph_->AddConstraint(constraint);
    }
  }

  // 当前只在真正插入关键帧后触发后端，避免每一帧 scan 都跑一次无意义优化。
  if (backend_->enabled() && result.insertion_required) {
    backend_->RunOptimization(*pose_graph_);
  }
  ++processed_scan_count_;
  if (debug_log_every_n_scans_ > 0 &&
      processed_scan_count_ % debug_log_every_n_scans_ == 0) {
    RCLCPP_INFO(get_logger(),
                "frontend status: scans=%d pose=(%.3f, %.3f, %.3f) "
                "keyframes=%zu insertion=%s",
                processed_scan_count_, result.local_pose.x, result.local_pose.y,
                result.local_pose.yaw, keyframe_poses_.size(),
                result.insertion_required ? "true" : "false");
  }
  PublishOutputs(result, msg->header.frame_id);
  if (result.insertion_required) {
    PublishActiveSubmap(rclcpp::Time(result.range_data.stamp));
    PublishLocalMap(rclcpp::Time(result.range_data.stamp));
    PublishGlobalMap(rclcpp::Time(result.range_data.stamp));
  }
}

void SimpleSlamNode::HandleOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = *msg;
}

void SimpleSlamNode::PublishOutputs(const LocalSlamResult2D& result,
                                    const std::string& scan_frame) {
  Pose2D published_pose = result.local_pose;
  Pose2D published_to_scan;
  if (!scan_frame.empty() && scan_frame != published_frame_) {
    try {
      const auto published_to_scan_tf = tf_buffer_->lookupTransform(
          published_frame_, scan_frame, tf2::TimePointZero);
      published_to_scan = PoseFromTransform(published_to_scan_tf.transform);
      published_pose =
          ComposePoses(result.local_pose, InversePose(published_to_scan));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "failed to lookup transform %s -> %s: %s",
                           published_frame_.c_str(), scan_frame.c_str(),
                           ex.what());
    }
  }

  Pose2D laser_odom_pose = frontend_->lidar_odom_pose();
  if (!scan_frame.empty() && scan_frame != published_frame_) {
    laser_odom_pose = ComposePoses(frontend_->lidar_odom_pose(),
                                   InversePose(published_to_scan));
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
  current_scan_cloud.width =
      static_cast<uint32_t>(result.range_data.returns.size());

  sensor_msgs::PointCloud2Modifier modifier(current_scan_cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(result.range_data.returns.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(current_scan_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(current_scan_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(current_scan_cloud, "z");
  for (const auto& point_in_scan : result.range_data.returns) {
    const auto point_in_published =
        TransformPoint(point_in_scan, published_to_scan);
    const auto point_in_map =
        TransformPoint(point_in_published, published_pose);
    *iter_x = static_cast<float>(point_in_map.x);
    *iter_y = static_cast<float>(point_in_map.y);
    *iter_z = 0.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  current_scan_cloud_pub_->publish(current_scan_cloud);

  Pose2D map_to_odom_pose = published_pose;
  if (latest_odom_ && latest_odom_->header.frame_id == odom_frame_ &&
      latest_odom_->child_frame_id == published_frame_) {
    map_to_odom_pose = ComposePoses(
        published_pose, InversePose(PoseFromOdometry(*latest_odom_)));
  } else if (odom_frame_ != published_frame_) {
    try {
      const auto odom_to_published_tf = tf_buffer_->lookupTransform(
          odom_frame_, published_frame_, tf2::TimePointZero);
      map_to_odom_pose = ComposePoses(
          published_pose,
          InversePose(PoseFromTransform(odom_to_published_tf.transform)));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "failed to lookup transform %s -> %s: %s",
                           odom_frame_.c_str(), published_frame_.c_str(),
                           ex.what());
    }
  }

  Pose2D tf_pose_to_publish = map_to_odom_pose;
  if (enable_tf_smoothing_) {
    if (!smoothed_tf_pose_) {
      smoothed_tf_pose_ = map_to_odom_pose;
    } else {
      smoothed_tf_pose_->x +=
          tf_translation_alpha_ * (map_to_odom_pose.x - smoothed_tf_pose_->x);
      smoothed_tf_pose_->y +=
          tf_translation_alpha_ * (map_to_odom_pose.y - smoothed_tf_pose_->y);
      const double yaw_delta =
          NormalizeAngle(map_to_odom_pose.yaw - smoothed_tf_pose_->yaw);
      smoothed_tf_pose_->yaw = NormalizeAngle(smoothed_tf_pose_->yaw +
                                              tf_rotation_alpha_ * yaw_delta);
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

void SimpleSlamNode::PublishKeyframeMarkers(const Pose2D& pose,
                                            const rclcpp::Time& stamp) {
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

nav_msgs::msg::OccupancyGrid SimpleSlamNode::BuildMergedMap(
    const std::vector<std::shared_ptr<Submap2D>>& submaps,
    const rclcpp::Time& stamp) const {
  nav_msgs::msg::OccupancyGrid merged_map;
  merged_map.header.frame_id = map_frame_;
  merged_map.header.stamp = stamp;
  merged_map.info.origin.orientation.w = 1.0;

  if (submaps.empty()) {
    return merged_map;
  }

  const double resolution = submaps.front()->GetResolution();

  double global_min_x = std::numeric_limits<double>::max();
  double global_min_y = std::numeric_limits<double>::max();
  double global_max_x = std::numeric_limits<double>::lowest();
  double global_max_y = std::numeric_limits<double>::lowest();
  // 地图边界由所有子图的边界框的并集确定。
  for (const auto& submap : submaps) {
    if (!submap) {
      continue;
    }

    const double submap_resolution = submap->GetResolution();
    // submap 分辨率不一致时暂时无法合并，后续可以考虑重采样。
    if (std::abs(submap_resolution - resolution) > 1e-9) {
      continue;
    }

    for (const auto& corner : submap->GetWorldCorners()) {
      global_min_x = std::min(global_min_x, corner.x);
      global_min_y = std::min(global_min_y, corner.y);
      global_max_x = std::max(global_max_x, corner.x);
      global_max_y = std::max(global_max_y, corner.y);
    }
  }

  if (global_min_x > global_max_x || global_min_y > global_max_y) {
    return merged_map;
  }
  // 获取地图长宽对应的栅格数量，向上取整确保能完整覆盖边界。
  const uint32_t global_width = static_cast<uint32_t>(
      std::ceil((global_max_x - global_min_x) / resolution));
  const uint32_t global_height = static_cast<uint32_t>(
      std::ceil((global_max_y - global_min_y) / resolution));

  merged_map.info.resolution = static_cast<float>(resolution);
  merged_map.info.width = global_width;
  merged_map.info.height = global_height;
  merged_map.info.origin.position.x = global_min_x;
  merged_map.info.origin.position.y = global_min_y;
  merged_map.info.origin.position.z = 0.0;
  merged_map.data.assign(static_cast<size_t>(global_width) * global_height, -1);

  auto flat_index = [global_width](const int x, const int y) -> size_t {
    return static_cast<size_t>(y) * static_cast<size_t>(global_width) +
           static_cast<size_t>(x);
  };

  for (const auto& submap : submaps) {
    if (!submap) {
      continue;
    }

    const double submap_resolution = submap->GetResolution();
    if (std::abs(submap_resolution - resolution) > 1e-9) {
      continue;
    }

    const int submap_width = submap->GetWidth();
    const int submap_height = submap->GetHeight();
    const auto submap_data = submap->ToOccupancyGridData();
    // 子图数据合并到全局地图时，未知值(-1)直接跳过。
    // 如果和已有值冲突，则保留更确定的那个值。
    for (int y = 0; y < submap_height; ++y) {
      for (int x = 0; x < submap_width; ++x) {
        const size_t submap_index =
            static_cast<size_t>(y) * static_cast<size_t>(submap_width) +
            static_cast<size_t>(x);
        const int8_t incoming_value = submap_data[submap_index];
        if (incoming_value < 0) {
          continue;
        }

        const Point2D cell_center = submap->GetCellCenterInWorld(x, y);
        const int global_x = static_cast<int>(
            std::floor((cell_center.x - global_min_x) / resolution));
        const int global_y = static_cast<int>(
            std::floor((cell_center.y - global_min_y) / resolution));
        // 越界的部分直接丢弃，后续可以考虑扩展地图边界。
        if (global_x < 0 || global_y < 0 ||
            global_x >= static_cast<int>(global_width) ||
            global_y >= static_cast<int>(global_height)) {
          continue;
        }

        int8_t& existing_value =
            merged_map.data[flat_index(global_x, global_y)];
        if (existing_value < 0) {
          existing_value = incoming_value;
          continue;
        }

        // OccupancyGrid 的 50 表示最不确定，离 50 越远说明越确定；
        // 因此这里比较的是“置信度”，而不是简单地比较数值大小。
        const int existing_confidence =
            std::abs(static_cast<int>(existing_value) - 50);
        const int incoming_confidence =
            std::abs(static_cast<int>(incoming_value) - 50);
        if (incoming_confidence > existing_confidence) {
          existing_value = incoming_value;
        }
      }
    }
  }

  return merged_map;
}

void SimpleSlamNode::PublishActiveSubmap(const rclcpp::Time& stamp) {
  const auto& active_submaps = frontend_->GetActiveSubmaps();
  if (active_submaps.empty()) {
    return;
  }
  const auto& submap = active_submaps.back();

  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.frame_id = map_frame_;
  grid_msg.header.stamp = stamp;
  grid_msg.info.resolution = submap->options().resolution;
  grid_msg.info.width = submap->options().width;
  grid_msg.info.height = submap->options().height;

  const Point2D lower_left = submap->GetLowerLeftCorner();
  grid_msg.info.origin.position.x = lower_left.x;
  grid_msg.info.origin.position.y = lower_left.y;
  grid_msg.info.origin.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, submap->global_pose().yaw);
  grid_msg.info.origin.orientation = tf2::toMsg(q);

  grid_msg.data = submap->ToOccupancyGridData();
  active_submap_pub_->publish(grid_msg);
}

void SimpleSlamNode::PublishLocalMap(const rclcpp::Time& stamp) {
  const auto& active_submaps = frontend_->GetActiveSubmaps();
  if (active_submaps.empty()) {
    return;
  }
  local_map_pub_->publish(BuildMergedMap(active_submaps, stamp));
}

void SimpleSlamNode::PublishGlobalMap(const rclcpp::Time& stamp) {
  std::vector<std::shared_ptr<Submap2D>> all_submaps =
      frontend_->GetFinishedSubmaps();
  const auto& active_submaps = frontend_->GetActiveSubmaps();
  all_submaps.insert(all_submaps.end(), active_submaps.begin(),
                     active_submaps.end());

  if (all_submaps.empty()) {
    return;
  }

  global_map_pub_->publish(BuildMergedMap(all_submaps, stamp));
}

}  // namespace simple_slam

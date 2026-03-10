#ifndef SIMPLE_SLAM__TYPES_HPP_
#define SIMPLE_SLAM__TYPES_HPP_

#include <cmath>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace simple_slam
{

// 基本二维点类型，统一用于激光点和网格投影。
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

// 系统内部统一使用的二维位姿表示。
struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// 一帧激光在前端预处理后的结果。
struct RangeData2D
{
  builtin_interfaces::msg::Time stamp;
  std::vector<Point2D> returns;
};

// 相关匹配搜索窗口参数，主要用于 scan-to-submap 粗匹配。
struct SearchParameters2D
{
  double linear_window = 0.3;
  double angular_window = 0.35;
  double linear_step = 0.05;
  double angular_step = 0.05;
  double translation_delta_cost_weight = 1.0;
  double rotation_delta_cost_weight = 0.2;
};

// 一次局部 SLAM 更新的输出结果。
struct LocalSlamResult2D
{
  bool valid = false;
  Pose2D local_pose;
  RangeData2D range_data;
  bool insertion_required = false;
  bool is_keyframe = false;
};

// 位姿图里存储的轨迹节点。
struct TrajectoryNode2D
{
  int id = -1;
  Pose2D local_pose;
  RangeData2D filtered_range_data;
};

inline geometry_msgs::msg::Pose ToRosPose(const Pose2D & pose)
{
  geometry_msgs::msg::Pose ros_pose;
  ros_pose.position.x = pose.x;
  ros_pose.position.y = pose.y;
  ros_pose.position.z = 0.0;
  return ros_pose;
}

// 把角度压回 [-pi, pi]，避免累计后出现跳变。
inline double NormalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// 在二维平面内旋转一个点。
inline Point2D RotatePoint(const Point2D & point, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Point2D{c * point.x - s * point.y, s * point.x + c * point.y};
}

// 把局部点变换到目标位姿所在坐标系。
inline Point2D TransformPoint(const Point2D & point, const Pose2D & pose)
{
  const auto rotated = RotatePoint(point, pose.yaw);
  return Point2D{rotated.x + pose.x, rotated.y + pose.y};
}

// 求二维位姿的逆。
inline Pose2D InversePose(const Pose2D & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const double x = -(c * pose.x + s * pose.y);
  const double y = -(-s * pose.x + c * pose.y);
  return Pose2D{x, y, NormalizeAngle(-pose.yaw)};
}

// 位姿复合：先施加 lhs，再施加 rhs。
inline Pose2D ComposePoses(const Pose2D & lhs, const Pose2D & rhs)
{
  const auto translated = TransformPoint(Point2D{rhs.x, rhs.y}, lhs);
  return Pose2D{translated.x, translated.y, NormalizeAngle(lhs.yaw + rhs.yaw)};
}

// 计算 from 到 to 的相对位姿。
inline Pose2D RelativePose(const Pose2D & from, const Pose2D & to)
{
  return ComposePoses(InversePose(from), to);
}

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__TYPES_HPP_

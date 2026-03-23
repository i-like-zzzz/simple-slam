#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "simple_slam/frontend/local_slam_frontend.hpp"

namespace simple_slam
{
namespace
{

sensor_msgs::msg::LaserScan MakeScan(const std::vector<float> & ranges)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -0.5F;
  scan.angle_increment = 0.1F;
  scan.range_min = 0.05F;
  scan.range_max = 20.0F;
  scan.ranges = ranges;
  return scan;
}

LocalSlamFrontend::Options MakeOptions(LidarOdomMatcherType matcher_type)
{
  LocalSlamFrontend::Options options;
  options.voxel_filter_size = 0.0;
  options.min_range_points_for_match = 4;
  options.scans_per_accumulation = 1;
  options.enable_map_update = true;
  options.active_submap_num_range_data = 10;
  options.keyframe_translation_threshold = 0.05;
  options.keyframe_rotation_threshold = 0.05;
  options.lidar_odom_matcher = matcher_type;
  options.lidar_odom_max_points = 64;
  options.lidar_odom_max_iterations = 20;
  options.lidar_odom_linear_window = 0.2;
  options.lidar_odom_angular_window = 0.2;
  options.lidar_odom_point_sigma = 0.2;
  options.scan_matcher.linear_window = 0.1;
  options.scan_matcher.angular_window = 0.1;
  options.scan_matcher.linear_step = 0.05;
  options.scan_matcher.angular_step = 0.05;
  return options;
}

TEST(LocalSlamFrontendTest, RejectsScansWithTooFewPoints)
{
  LocalSlamFrontend frontend(MakeOptions(LidarOdomMatcherType::kPointToPointIcp));
  const auto sparse_scan = MakeScan({
      1.0F,
      std::numeric_limits<float>::infinity(),
      1.2F});

  const auto result = frontend.AddScan(sparse_scan, nullptr);

  EXPECT_FALSE(result.valid);
}

TEST(LocalSlamFrontendTest, SupportsAllLidarOdomMatchersOnRepeatedScans)
{
  std::vector<float> dense_ranges;
  dense_ranges.reserve(32);
  for (int index = 0; index < 32; ++index) {
    dense_ranges.push_back(1.0F + 0.03F * static_cast<float>(index));
  }
  const auto scan = MakeScan(dense_ranges);

  for (const auto matcher_type : {
         LidarOdomMatcherType::kPointToPointIcp,
         LidarOdomMatcherType::kGeneralizedIcp,
         LidarOdomMatcherType::kCorrelative})
  {
    LocalSlamFrontend frontend(MakeOptions(matcher_type));

    const auto first_result = frontend.AddScan(scan, nullptr);
    const auto second_result = frontend.AddScan(scan, nullptr);

    EXPECT_TRUE(first_result.valid);
    EXPECT_TRUE(second_result.valid);
    EXPECT_TRUE(std::isfinite(second_result.local_pose.x));
    EXPECT_TRUE(std::isfinite(second_result.local_pose.y));
    EXPECT_TRUE(std::isfinite(second_result.local_pose.yaw));
  }
}

}  // namespace
}  // namespace simple_slam

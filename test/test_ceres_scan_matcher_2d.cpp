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

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "simple_slam/frontend/ceres_scan_matcher_2d.hpp"

namespace simple_slam {
namespace {

std::vector<Point2D> RotatePoints(const std::vector<Point2D>& points,
                                  const double yaw) {
  std::vector<Point2D> rotated_points;
  rotated_points.reserve(points.size());

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  for (const auto& point : points) {
    rotated_points.push_back(Point2D{cos_yaw * point.x - sin_yaw * point.y,
                                     sin_yaw * point.x + cos_yaw * point.y});
  }
  return rotated_points;
}

TEST(CeresScanMatcher2DTest, MatchesPerfectlyAlignedScans) {
  const std::vector<Point2D> points = {
      {1.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {1.0, 2.0}, {0.5, 1.5},
  };

  CeresScanMatcher2D::Options options;
  options.max_correspondence_distance = 1.0;
  options.huber_scale = 0.1;
  options.max_num_iterations = 20;

  CeresScanMatcher2D matcher(options);

  const Pose2D initial_relative_pose{0.0, 0.0, 0.0};
  const Pose2D matched_pose =
      matcher.Match(points, points, initial_relative_pose);

  EXPECT_NEAR(matched_pose.x, 0.0, 1e-6);
  EXPECT_NEAR(matched_pose.y, 0.0, 1e-6);
  EXPECT_NEAR(matched_pose.yaw, 0.0, 1e-6);
}

TEST(CeresScanMatcher2DTest, RecoversSmallRotation) {
  const std::vector<Point2D> previous_points = {
      {1.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {1.0, 2.0}, {0.5, 1.5},
  };
  const double yaw_delta = -0.2;
  const std::vector<Point2D> current_points =
      RotatePoints(previous_points, yaw_delta);

  CeresScanMatcher2D::Options options;
  options.max_correspondence_distance = 1.0;
  options.huber_scale = 0.1;
  options.max_num_iterations = 20;

  CeresScanMatcher2D matcher(options);

  const Pose2D initial_relative_pose{0.0, 0.0, 0.0};
  const Pose2D matched_pose =
      matcher.Match(current_points, previous_points, initial_relative_pose);

  EXPECT_NEAR(matched_pose.x, 0.0, 1e-2);
  EXPECT_NEAR(matched_pose.y, 0.0, 1e-2);
  EXPECT_NEAR(matched_pose.yaw, 0.2, 5e-2);
}

}  // namespace
}  // namespace simple_slam

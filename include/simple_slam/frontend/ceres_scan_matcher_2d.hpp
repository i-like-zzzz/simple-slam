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

#ifndef SIMPLE_SLAM__FRONTEND__CERES_SCAN_MATCHER_2D_HPP_
#define SIMPLE_SLAM__FRONTEND__CERES_SCAN_MATCHER_2D_HPP_

#include <vector>

#include "simple_slam/types.hpp"

namespace simple_slam
{

class CeresScanMatcher2D
{
public:
  struct Options
  {
    double max_correspondence_distance = 0.3;
    double huber_scale = 0.1;
    int max_num_iterations = 20;
  };

  explicit CeresScanMatcher2D(Options options);

  Pose2D Match(
    const std::vector<Point2D> & current_points,
    const std::vector<Point2D> & previous_points,
    const Pose2D & initial_relative_pose) const;

private:
  Options options_;
};

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__FRONTEND__CERES_SCAN_MATCHER_2D_HPP_

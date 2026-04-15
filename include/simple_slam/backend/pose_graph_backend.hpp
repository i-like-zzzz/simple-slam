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

#ifndef SIMPLE_SLAM__BACKEND__POSE_GRAPH_BACKEND_HPP_
#define SIMPLE_SLAM__BACKEND__POSE_GRAPH_BACKEND_HPP_

#include "simple_slam/types.hpp"
#include "simple_slam/optimization/pose_graph_2d.hpp"

namespace simple_slam {

// 后端接口先独立出来，前端稳定后可以把回环、约束构建和优化逐步填进来。
class PoseGraphBackend {
 public:
  struct Options {
    bool enable_backend = false;
  };

  explicit PoseGraphBackend(Options options);

  // 当前入口已经接好了，但实现仍停留在“遍历约束、计算误差”的阶段。
  // 下一步会在这里逐步补：
  // 1. 读取 nodes / submaps / constraints
  // 2. 求解优化后的 node/submap pose
  // 3. 回写 PoseGraph2D，驱动地图重新发布
  void RunOptimization(PoseGraph2D& pose_graph);

  bool enabled() const;

 private:
  Options options_;
};

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__BACKEND__POSE_GRAPH_BACKEND_HPP_

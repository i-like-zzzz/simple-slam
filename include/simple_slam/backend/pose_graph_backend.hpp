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

namespace simple_slam {

// 后端接口先独立出来，前端稳定后可以把回环、约束构建和优化逐步填进来。
class PoseGraphBackend {
 public:
  struct Options {
    bool enable_backend = false;
  };

  explicit PoseGraphBackend(Options options);

  void AddLocalSlamResult(const LocalSlamResult2D& result);
  bool enabled() const;

 private:
  Options options_;
};

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__BACKEND__POSE_GRAPH_BACKEND_HPP_

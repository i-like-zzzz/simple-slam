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

#include "simple_slam/backend/pose_graph_backend.hpp"

namespace simple_slam {

PoseGraphBackend::PoseGraphBackend(Options options) : options_(options) {}

void PoseGraphBackend::AddLocalSlamResult(const LocalSlamResult2D&) {
  // 当前版本先保留后端接入口，后续在这里接约束搜索和非线性优化。
}

bool PoseGraphBackend::enabled() const { return options_.enable_backend; }

}  // namespace simple_slam

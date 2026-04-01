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

#include "simple_slam/optimization/pose_graph_2d.hpp"

#include <algorithm>

namespace simple_slam {

PoseGraph2D::PoseGraph2D(Options options) : options_(options) {}

void PoseGraph2D::AddNode(const LocalSlamResult2D& result) {
  if (!result.valid) {
    return;
  }

  nodes_.push_back(
      TrajectoryNode2D{next_node_id_++, result.local_pose, result.range_data});
}

const std::vector<TrajectoryNode2D>& PoseGraph2D::nodes() const {
  return nodes_;
}

const std::vector<std::shared_ptr<Submap2D>>& PoseGraph2D::submaps() const {
  return submaps_;
}

void PoseGraph2D::RegisterSubmaps(
    const std::vector<std::shared_ptr<Submap2D>>& active_submaps) {
  // 只登记新出现的子图，避免每帧都重复压入同一份 shared_ptr。
  for (const auto& submap : active_submaps) {
    const auto existing =
        std::find_if(submaps_.begin(), submaps_.end(),
                     [submap](const std::shared_ptr<Submap2D>& candidate) {
                       return candidate->id() == submap->id();
                     });
    if (existing == submaps_.end()) {
      submaps_.push_back(submap);
    }
  }
}

}  // namespace simple_slam

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

int PoseGraph2D::AddNode(const LocalSlamResult2D& result) {
  if (!result.valid) {
    return -1;
  }
  const int node_id = next_node_id_++;
  nodes_.push_back(
      TrajectoryNode2D{node_id, result.local_pose, result.range_data});
  return node_id;
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

void PoseGraph2D::AddConstraint(const Constraint2D& constraint) {
  constraints_.push_back(constraint);
}

const std::vector<Constraint2D>& PoseGraph2D::constraints() const {
  return constraints_;
}

void PoseGraph2D::UpdateNodePose(int node_id, const Pose2D& pose) {
  const auto node = std::find_if(nodes_.begin(), nodes_.end(),
                             [node_id](const TrajectoryNode2D& node) {
                               return node.id == node_id;
                             });
  if(node == nodes_.end()) {
    return;
  }
  node->local_pose = pose;
}

void PoseGraph2D::UpdateSubmapPose(int submap_id, const Pose2D& pose) {
  const auto submap = std::find_if(submaps_.begin(), submaps_.end(),
                             [submap_id](const std::shared_ptr<Submap2D>& submap) {
                               return submap && submap->id() == submap_id;
                             });
  if(submap == submaps_.end() || !(*submap)) {
    return;
  }
  (*submap)->SetGlobalPose(pose);
}

}  // namespace simple_slam

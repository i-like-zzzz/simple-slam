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

#include <algorithm>

#include "simple_slam/backend/pose_graph_backend.hpp"

namespace simple_slam {

namespace {
const TrajectoryNode2D* FindNodeById(
    const std::vector<TrajectoryNode2D>& nodes, int node_id) {
  const auto it = std::find_if(
      nodes.begin(), nodes.end(),
      [node_id](const TrajectoryNode2D& node) { return node.id == node_id; });
  if (it == nodes.end()) {
    return nullptr;
  }
  return &(*it);
}

std::shared_ptr<Submap2D> FindSubmapById(
    const std::vector<std::shared_ptr<Submap2D>>& submaps,
    const int submap_id) {
  const auto it = std::find_if(
      submaps.begin(), submaps.end(),
      [submap_id](const std::shared_ptr<Submap2D>& submap) {
        return submap && submap->id() == submap_id;
      });
  if (it == submaps.end()) {
    return nullptr;
  }
  return *it;
}

}  // namespace

PoseGraphBackend::PoseGraphBackend(Options options) : options_(options) {}

void PoseGraphBackend::RunOptimization(PoseGraph2D& pose_graph) {
  // 当前版本还不是“真正的 pose graph optimizer”：
  // 它只是把约束链路完整走一遍，验证 node / submap / constraint
  // 三层数据已经能被后端统一读取。
  if (!enabled()) {
    return;
  }

  const auto& nodes = pose_graph.nodes();
  const auto& submaps = pose_graph.submaps();
  const auto& constraints = pose_graph.constraints();
  if (nodes.empty() || submaps.empty() || constraints.empty()) {
    return;
  }

  for (const auto& constraint : constraints) {
    const TrajectoryNode2D* node_ptr = FindNodeById(nodes, constraint.node_id);
    const std::shared_ptr<Submap2D> submap_ptr =
        FindSubmapById(submaps, constraint.submap_id);
    if (node_ptr == nullptr || submap_ptr == nullptr) {
      continue;
    }

    // predicted_relative_pose 表示按“当前图状态”推导出的 T_submap_node，
    // measured_relative_pose 则是建图时记录下来的观测值。
    // 后续真正优化时，本质上就是让这两者尽量接近。
    const Pose2D predicted_relative_pose =
        RelativePose(submap_ptr->global_pose(), node_ptr->local_pose);
    const Pose2D measured_relative_pose = constraint.relative_pose;
    const double translation_error = std::hypot(
        predicted_relative_pose.x - measured_relative_pose.x,
        predicted_relative_pose.y - measured_relative_pose.y);
    const double rotation_error = std::abs(
        NormalizeAngle(predicted_relative_pose.yaw -
                       measured_relative_pose.yaw));

    // 这里暂时只计算误差，不回写 pose_graph。
    // 等你开始补真正的后端时，可以先从“固定 node，只调整 submap pose”
    // 这一版最小实现入手，再逐步扩展到同时优化 node 和 submap。
    // RCLCPP_INFO(rclcpp::get_logger("PoseGraphBackend"),
    //             "Constraint check: node_id=%d submap_id=%d "
    //             "predicted_relative_pose=(%.3f, %.3f, %.3f) "
    //             "measured_relative_pose=(%.3f, %.3f, %.3f) "
    //             "translation_error=%.3f rotation_error=%.3f",
    //             constraint.node_id, constraint.submap_id,
    //             predicted_relative_pose.x, predicted_relative_pose.y,
    //             predicted_relative_pose.yaw, measured_relative_pose.x,
    //             measured_relative_pose.y, measured_relative_pose.yaw,
    //             translation_error, rotation_error);
    (void)translation_error;
    (void)rotation_error;
  }
}

bool PoseGraphBackend::enabled() const { return options_.enable_backend; }

}  // namespace simple_slam

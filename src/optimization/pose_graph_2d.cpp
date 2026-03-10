#include "simple_slam/optimization/pose_graph_2d.hpp"

#include <algorithm>

namespace simple_slam
{

PoseGraph2D::PoseGraph2D(Options options)
: options_(options)
{
}

void PoseGraph2D::AddNode(const LocalSlamResult2D & result)
{
  if (!result.valid) {
    return;
  }

  nodes_.push_back(TrajectoryNode2D{
    next_node_id_++,
    result.local_pose,
    result.range_data});
}

const std::vector<TrajectoryNode2D> & PoseGraph2D::nodes() const
{
  return nodes_;
}

const std::vector<std::shared_ptr<Submap2D>> & PoseGraph2D::submaps() const
{
  return submaps_;
}

void PoseGraph2D::RegisterSubmaps(const std::vector<std::shared_ptr<Submap2D>> & active_submaps)
{
  // 只登记新出现的子图，避免每帧都重复压入同一份 shared_ptr。
  for (const auto & submap : active_submaps) {
    const auto existing = std::find_if(
      submaps_.begin(), submaps_.end(),
      [submap](const std::shared_ptr<Submap2D> & candidate) {
        return candidate->id() == submap->id();
      });
    if (existing == submaps_.end()) {
      submaps_.push_back(submap);
    }
  }
}

}  // namespace simple_slam

#ifndef SIMPLE_SLAM__OPTIMIZATION__POSE_GRAPH_2D_HPP_
#define SIMPLE_SLAM__OPTIMIZATION__POSE_GRAPH_2D_HPP_

#include <memory>
#include <vector>

#include "simple_slam/mapping/submap_2d.hpp"
#include "simple_slam/types.hpp"

namespace simple_slam
{

// 当前先作为位姿图数据容器，后面再逐步接后端优化。
class PoseGraph2D
{
public:
  struct Options
  {
    int active_submap_num_range_data = 90;
  };

  explicit PoseGraph2D(Options options);

  // 添加一个新的轨迹节点。
  void AddNode(const LocalSlamResult2D & result);
  const std::vector<TrajectoryNode2D> & nodes() const;
  const std::vector<std::shared_ptr<Submap2D>> & submaps() const;

  // 把前端创建出的活动子图登记到位姿图容器里。
  void RegisterSubmaps(const std::vector<std::shared_ptr<Submap2D>> & active_submaps);

private:
  Options options_;
  int next_node_id_ = 0;
  std::vector<TrajectoryNode2D> nodes_;
  std::vector<std::shared_ptr<Submap2D>> submaps_;
};

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__OPTIMIZATION__POSE_GRAPH_2D_HPP_

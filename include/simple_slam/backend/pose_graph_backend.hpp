#ifndef SIMPLE_SLAM__BACKEND__POSE_GRAPH_BACKEND_HPP_
#define SIMPLE_SLAM__BACKEND__POSE_GRAPH_BACKEND_HPP_

#include "simple_slam/types.hpp"

namespace simple_slam
{

// 后端接口先独立出来，前端稳定后可以把回环、约束构建和优化逐步填进来。
class PoseGraphBackend
{
public:
  struct Options
  {
    bool enable_backend = false;
  };

  explicit PoseGraphBackend(Options options);

  void AddLocalSlamResult(const LocalSlamResult2D & result);
  bool enabled() const;

private:
  Options options_;
};

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__BACKEND__POSE_GRAPH_BACKEND_HPP_

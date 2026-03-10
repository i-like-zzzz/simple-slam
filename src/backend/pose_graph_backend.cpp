#include "simple_slam/backend/pose_graph_backend.hpp"

namespace simple_slam
{

PoseGraphBackend::PoseGraphBackend(Options options)
: options_(options)
{
}

void PoseGraphBackend::AddLocalSlamResult(const LocalSlamResult2D &)
{
  // 当前版本先保留后端接入口，后续在这里接约束搜索和非线性优化。
}

bool PoseGraphBackend::enabled() const
{
  return options_.enable_backend;
}

}  // namespace simple_slam

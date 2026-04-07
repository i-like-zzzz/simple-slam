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

#include "simple_slam/mapping/submap_2d.hpp"

#include <algorithm>
#include <cmath>

namespace simple_slam {

namespace {

// 占据概率转 log-odds，方便反复累计更新。
double ProbabilityToLogOdds(const double probability) {
  return std::log(probability / (1.0 - probability));
}

// 内部存的是 log-odds，对外查询时再还原成概率。
double LogOddsToProbability(const double log_odds) {
  const double odds = std::exp(log_odds);
  return odds / (1.0 + odds);
}

}  // namespace

Submap2D::Submap2D(int id, Options options, const Pose2D& initial_pose)
    : id_(id),
      options_(options),
      global_pose_(initial_pose),
      local_map_origin_{-0.5 * options.width * options.resolution , 
                        -0.5 * options.height * options.resolution},
      log_odds_cells_(static_cast<size_t>(options.width * options.height), 0.0),
      known_cells_(static_cast<size_t>(options.width * options.height), false) {
}

void Submap2D::InsertRangeData(const RangeData2D& range_data,
                               const Pose2D& local_pose) {
  // 子图内部统一使用局部地图坐标，便于后面接后端优化后的位姿修正。
  const Point2D sensor_origin_world{local_pose.x, local_pose.y};
  const Point2D sensor_origin_local = WorldToLocal(sensor_origin_world);

  for (const auto& hit_in_sensor : range_data.returns) {
    const Point2D hit_in_world = TransformPoint(hit_in_sensor, local_pose);
    const Point2D hit_in_local = WorldToLocal(hit_in_world);
    CastRay(sensor_origin_local, hit_in_local);
  }
  ++num_insertions_;
}

bool Submap2D::IsFinished() const {
  return num_insertions_ > 0 &&
         num_insertions_ >= options_.num_range_data_limit;
}

int Submap2D::id() const { return id_; }

int Submap2D::num_insertions() const { return num_insertions_; }

const Pose2D& Submap2D::global_pose() const { return global_pose_; }

void Submap2D::SetGlobalPose(const Pose2D& pose) {
  global_pose_ = pose;
}
// 获取子图左下角在世界坐标系中的位置。
Point2D Submap2D::GetLowerLeftCorner() const {
  return LocalToWorld(local_map_origin_);
}

double Submap2D::GetProbability(const Point2D& world_point) const {
  const auto local_point = WorldToLocal(world_point);
  const auto index = LocalToGrid(local_point);
  if (!index.has_value()) {
    return 0.1;
  }
  return LogOddsToProbability(log_odds_cells_[ToFlatIndex(*index)]);
}

bool Submap2D::HasSufficientData() const { return num_insertions_ >= 3; }

const Submap2D::Options& Submap2D::options() const { return options_; }

int Submap2D::ToFlatIndex(const GridIndex& index) const {
  return index.y * options_.width + index.x;
}

bool Submap2D::IsInside(const GridIndex& index) const {
  return index.x >= 0 && index.x < options_.width && index.y >= 0 &&
         index.y < options_.height;
}

Point2D Submap2D::WorldToLocal(const Point2D& world_point) const {
  return TransformPoint(world_point, InversePose(global_pose_));
}

Point2D Submap2D::LocalToWorld(const Point2D& local_point) const {
  return TransformPoint(local_point, global_pose_);
}

std::optional<Submap2D::GridIndex> Submap2D::LocalToGrid(
    const Point2D& local_point) const {
    const int cell_x = static_cast<int>(
      std::floor((local_point.x - local_map_origin_.x) / options_.resolution));
      const int cell_y = static_cast<int>(
        std::floor((local_point.y - local_map_origin_.y) / options_.resolution));
    GridIndex index{cell_x, cell_y};
    if(!IsInside(index)) {
      return std::nullopt;
    }
    return index;
}


Point2D Submap2D::GridToLocal(const GridIndex& index) const {
  return Point2D{local_map_origin_.x + 
                 (static_cast<double>(index.x) + 0.5) * options_.resolution,
                 local_map_origin_.y +
                 (static_cast<double>(index.y) + 0.5) * options_.resolution};
}
// 更新栅格。
void Submap2D::UpdateCell(const GridIndex& index, const double delta) {
  if (!IsInside(index)) {
    return;
  }
  // 限幅避免单个栅格在长时间运行后数值过饱和。
  double& log_odds = log_odds_cells_[ToFlatIndex(index)];
  known_cells_[ToFlatIndex(index)] = true;
  log_odds = std::clamp(log_odds + delta, -4.0, 4.0);
}

void Submap2D::CastRay(const Point2D& start_local, const Point2D& end_local) {
  const auto start_index = LocalToGrid(start_local);
  const auto end_index = LocalToGrid(end_local);
  if (!start_index.has_value() || !end_index.has_value()) {
    return;
  }

  const int dx = end_index->x - start_index->x;
  const int dy = end_index->y - start_index->y;
  const int steps = std::max(std::abs(dx), std::abs(dy));
  if (steps == 0) {
    UpdateCell(*end_index, ProbabilityToLogOdds(options_.hit_probability));
    return;
  }

  const double x_increment =
      static_cast<double>(dx) / static_cast<double>(steps);
  const double y_increment =
      static_cast<double>(dy) / static_cast<double>(steps);
  double x = static_cast<double>(start_index->x);
  double y = static_cast<double>(start_index->y);

  // 先把经过的空闲栅格压低，再把终点作为命中更新。
  for (int step = 0; step < steps; ++step) {
    UpdateCell(GridIndex{static_cast<int>(std::lround(x)),
                         static_cast<int>(std::lround(y))},
               ProbabilityToLogOdds(options_.miss_probability));
    x += x_increment;
    y += y_increment;
  }

  UpdateCell(*end_index, ProbabilityToLogOdds(options_.hit_probability));
}

std::vector<int8_t> Submap2D::ToOccupancyGridData() const {
  std::vector<int8_t> occupancy_data;
  occupancy_data.reserve(log_odds_cells_.size());

  for (size_t i = 0; i < log_odds_cells_.size(); i++) {
    if (!known_cells_[i]) {
      occupancy_data.push_back(-1);
      continue;
    }

    const double probability = LogOddsToProbability(log_odds_cells_[i]);
    int occupancy_value = static_cast<int>(
        std::lround(std::clamp(probability, 0.0, 1.0) * 100.0));
    occupancy_data.push_back(static_cast<int8_t>(occupancy_value));
  }

  return occupancy_data;
}

}  // namespace simple_slam

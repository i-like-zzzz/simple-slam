#include "simple_slam/mapping/submap_2d.hpp"

#include <algorithm>
#include <cmath>

namespace simple_slam
{

namespace
{

double ProbabilityToLogOdds(const double probability)
{
  return std::log(probability / (1.0 - probability));
}

double LogOddsToProbability(const double log_odds)
{
  const double odds = std::exp(log_odds);
  return odds / (1.0 + odds);
}

}  // namespace

Submap2D::Submap2D(int id, Options options, const Pose2D & initial_pose)
: id_(id),
  options_(options),
  origin_(initial_pose),
  map_center_{initial_pose.x, initial_pose.y},
  log_odds_cells_(static_cast<size_t>(options.width * options.height), 0.0)
{
}

void Submap2D::InsertRangeData(const RangeData2D & range_data, const Pose2D & local_pose)
{
  // 子图内部统一使用局部地图坐标，便于后面接后端优化后的位姿修正。
  const Point2D sensor_origin{local_pose.x, local_pose.y};
  for (const auto & hit_in_sensor : range_data.returns) {
    const Point2D hit_in_world = TransformPoint(hit_in_sensor, local_pose);
    CastRay(sensor_origin, hit_in_world);
  }
  ++num_insertions_;
}

bool Submap2D::IsFinished() const
{
  return num_insertions_ > 0 && num_insertions_ >= options_.num_range_data_limit;
}

int Submap2D::id() const
{
  return id_;
}

int Submap2D::num_insertions() const
{
  return num_insertions_;
}

const Pose2D & Submap2D::origin() const
{
  return origin_;
}

double Submap2D::GetProbability(const Point2D & world_point) const
{
  const auto index = WorldToGrid(world_point);
  if (!index.has_value()) {
    return 0.1;
  }
  return LogOddsToProbability(log_odds_cells_[ToFlatIndex(*index)]);
}

bool Submap2D::HasSufficientData() const
{
  return num_insertions_ >= 3;
}

const Submap2D::Options & Submap2D::options() const
{
  return options_;
}

int Submap2D::ToFlatIndex(const GridIndex & index) const
{
  return index.y * options_.width + index.x;
}

bool Submap2D::IsInside(const GridIndex & index) const
{
  return index.x >= 0 && index.x < options_.width && index.y >= 0 && index.y < options_.height;
}

std::optional<Submap2D::GridIndex> Submap2D::WorldToGrid(const Point2D & world_point) const
{
  const double origin_x = map_center_.x - 0.5 * static_cast<double>(options_.width) * options_.resolution;
  const double origin_y = map_center_.y - 0.5 * static_cast<double>(options_.height) * options_.resolution;
  const int cell_x = static_cast<int>(std::floor((world_point.x - origin_x) / options_.resolution));
  const int cell_y = static_cast<int>(std::floor((world_point.y - origin_y) / options_.resolution));
  GridIndex index{cell_x, cell_y};
  if (!IsInside(index)) {
    return std::nullopt;
  }
  return index;
}

Point2D Submap2D::GridToWorld(const GridIndex & index) const
{
  const double origin_x = map_center_.x - 0.5 * static_cast<double>(options_.width) * options_.resolution;
  const double origin_y = map_center_.y - 0.5 * static_cast<double>(options_.height) * options_.resolution;
  return Point2D{
    origin_x + (static_cast<double>(index.x) + 0.5) * options_.resolution,
    origin_y + (static_cast<double>(index.y) + 0.5) * options_.resolution};
}

void Submap2D::UpdateCell(const GridIndex & index, const double delta)
{
  if (!IsInside(index)) {
    return;
  }
  double & log_odds = log_odds_cells_[ToFlatIndex(index)];
  log_odds = std::clamp(log_odds + delta, -4.0, 4.0);
}

void Submap2D::CastRay(const Point2D & start, const Point2D & end)
{
  const auto start_index = WorldToGrid(start);
  const auto end_index = WorldToGrid(end);
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

  const double x_increment = static_cast<double>(dx) / static_cast<double>(steps);
  const double y_increment = static_cast<double>(dy) / static_cast<double>(steps);
  double x = static_cast<double>(start_index->x);
  double y = static_cast<double>(start_index->y);

  // 先把经过的空闲栅格压低，再把终点作为命中更新。
  for (int step = 0; step < steps; ++step) {
    UpdateCell(
      GridIndex{static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y))},
      ProbabilityToLogOdds(options_.miss_probability));
    x += x_increment;
    y += y_increment;
  }

  UpdateCell(*end_index, ProbabilityToLogOdds(options_.hit_probability));
}

}  // namespace simple_slam

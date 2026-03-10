#ifndef SIMPLE_SLAM__MAPPING__SUBMAP_2D_HPP_
#define SIMPLE_SLAM__MAPPING__SUBMAP_2D_HPP_

#include <optional>
#include <vector>

#include "simple_slam/types.hpp"

namespace simple_slam
{

class Submap2D
{
public:
  struct Options
  {
    double resolution = 0.05;
    int width = 400;
    int height = 400;
    int num_range_data_limit = 90;
    double hit_probability = 0.7;
    double miss_probability = 0.49;
  };

  Submap2D(int id, Options options, const Pose2D & initial_pose);

  void InsertRangeData(const RangeData2D & range_data, const Pose2D & local_pose);
  bool IsFinished() const;
  int id() const;
  int num_insertions() const;
  const Pose2D & origin() const;
  double GetProbability(const Point2D & world_point) const;
  bool HasSufficientData() const;
  const Options & options() const;

private:
  struct GridIndex
  {
    int x = 0;
    int y = 0;
  };

  int ToFlatIndex(const GridIndex & index) const;
  bool IsInside(const GridIndex & index) const;
  std::optional<GridIndex> WorldToGrid(const Point2D & world_point) const;
  Point2D GridToWorld(const GridIndex & index) const;
  void UpdateCell(const GridIndex & index, double delta);
  void CastRay(const Point2D & start, const Point2D & end);

  int id_;
  Options options_;
  int num_insertions_ = 0;
  Pose2D origin_;
  Point2D map_center_;
  std::vector<double> log_odds_cells_;
};

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__MAPPING__SUBMAP_2D_HPP_

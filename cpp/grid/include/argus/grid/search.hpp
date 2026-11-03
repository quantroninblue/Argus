#pragma once

#include <cstdint>
#include <vector>

#include "argus/grid/cell.hpp"

namespace argus::grid {

// A dense box of cells prepared for path search, with a distance field. Planar grids have one
// layer and put every cell centre at height `z`. Indices are local: 0 at the box corner.
class SearchGrid {
 public:
  // `origin` is the global cell index of local (0, 0, 0); its z is ignored for planar grids.
  void assign(const Index3& origin, const Index3& dims, double res, bool planar, double z,
              std::vector<Cell> cells);

  bool planar() const { return planar_; }
  const Index3& dims() const { return dims_; }
  const Index3& origin() const { return origin_; }
  double resolution() const { return res_; }

  bool contains(const Index3& i) const;
  Index3 local_of(const Vec3& p) const;
  Vec3 center_of(const Index3& i) const;
  size_t linear(const Index3& i) const {
    return (static_cast<size_t>(i.x()) * dims_.y() + i.y()) * dims_.z() + i.z();
  }

  Cell state(const Index3& i) const { return cells_[linear(i)]; }
  // Distance in metres between this cell's centre and the nearest occupied cell's centre.
  double clearance(const Index3& i) const { return clearance_[linear(i)]; }

  // Outside the box: Unknown and no known obstacle.
  Cell state_at(const Vec3& p) const;
  double clearance_at(const Vec3& p) const;

  Vec3 lower() const;
  Vec3 upper() const;

  static constexpr double kFar = 1e3;

 private:
  bool planar_ = true;
  Index3 dims_ = Index3::Zero();
  Index3 origin_ = Index3::Zero();
  double res_ = 0.1;
  double z_ = 0.0;
  std::vector<Cell> cells_;
  std::vector<float> clearance_;
};

// Euclidean distance transform. `occupied` is laid out like SearchGrid::linear. Returns metres,
// SearchGrid::kFar where there is no occupied cell at all.
std::vector<float> distance_field(const std::vector<uint8_t>& occupied, const Index3& dims, double res);

struct PathRequest {
  Vec3 start = Vec3::Zero();
  Vec3 goal = Vec3::Zero();
  // Cells closer than this to an obstacle are not entered (except the start cell).
  double min_clearance_m = 0.0;
  // Cells closer than this cost more, so paths keep away from obstacles when they can.
  double comfort_clearance_m = 0.0;
  double unknown_penalty = 1.0;
  int max_expansions = 200000;
  // When set (two points or more, and a width above zero), cells farther than `corridor_m`
  // from this polyline are not entered: a path that stays near a route given from outside.
  std::vector<Vec3> corridor;
  double corridor_m = 0.0;
};

// Distance from `p` to the polyline `line` (in the plane when `planar`).
double distance_to_polyline(const std::vector<Vec3>& line, const Vec3& p, bool planar);

struct PathResult {
  std::vector<Vec3> points;
  // False when the goal (clipped to the box) could not be reached; the path then ends at the
  // explored cell closest to it.
  bool reached = false;
  // Not reached and every cell reachable from the start was searched: known obstacles cut the
  // goal off. False when the search stopped at its expansion budget instead.
  bool exhausted = false;
  // The goal cell itself is occupied or too close to an obstacle to enter.
  bool goal_blocked = false;
};

// A* over a search grid: 8-connected for planar grids, 26-connected otherwise.
class PathSearch {
 public:
  PathResult find(const SearchGrid& grid, const PathRequest& request);

 private:
  std::vector<float> g_;
  std::vector<int> parent_;
  std::vector<uint32_t> seen_;
  std::vector<uint8_t> closed_;
  uint32_t generation_ = 0;
};

}  // namespace argus::grid

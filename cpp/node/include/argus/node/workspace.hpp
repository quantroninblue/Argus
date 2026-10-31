#pragma once

#include <vector>

#include "argus/node/body.hpp"
#include "argus/node/local_map.hpp"

namespace argus::node {

// The local map reduced to what one body needs for planning, with a distance field. Planar
// bodies get a 2D grid of columns over their height band; aerial bodies the voxel grid.
// Indices here are local: 0 at the window corner.
class Workspace {
 public:
  void build(const LocalMap& map, const BodyVolume& body, const Pose4& pose);

  bool planar() const { return planar_; }
  const Index3& dims() const { return dims_; }
  double resolution() const { return res_; }

  bool contains(const Index3& i) const;
  Index3 local_of(const Vec3& p) const;
  // Planar workspaces put every centre at the height of the base they were built for.
  Vec3 center_of(const Index3& i) const;
  size_t linear(const Index3& i) const {
    return (static_cast<size_t>(i.x()) * dims_.y() + i.y()) * dims_.z() + i.z();
  }

  Cell state(const Index3& i) const { return cells_[linear(i)]; }
  // Distance in metres between this cell's centre and the nearest occupied cell's centre.
  double clearance(const Index3& i) const { return clearance_[linear(i)]; }

  // Outside the window: Unknown and no known obstacle.
  Cell state_at(const Vec3& p) const;
  double clearance_at(const Vec3& p) const;

  // Window bounds in odom, for clipping goals.
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

// Euclidean distance transform. `occupied` is laid out like Workspace::linear. Returns metres.
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
};

struct PathResult {
  std::vector<Vec3> points;
  // False when the goal (clipped to the window) could not be reached; the path then ends at the
  // explored cell closest to it.
  bool reached = false;
};

// A* over a workspace: 8-connected for planar bodies, 26-connected for aerial ones.
class PathSearch {
 public:
  PathResult find(const Workspace& ws, const PathRequest& request);

 private:
  std::vector<float> g_;
  std::vector<int> parent_;
  std::vector<uint32_t> seen_;
  std::vector<uint8_t> closed_;
  uint32_t generation_ = 0;
};

}  // namespace argus::node

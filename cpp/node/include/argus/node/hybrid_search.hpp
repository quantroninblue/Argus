#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "argus/grid/search.hpp"
#include "argus/node/types.hpp"

namespace argus::node {

// Hybrid A* for planar bodies with a minimum turn radius (Dolgov, Thrun, Montemerlo and Diebel,
// "Practical Search Techniques in Path Planning for Autonomous Driving", 2008; the same scheme
// as Nav2's Smac Hybrid planner). States are continuous poses binned by grid cell and heading;
// each step is a forward or reverse arc at the tightest turn either way, or straight. Near the
// goal the search tries a direct turn-then-straight curve, which is the shortest form when the
// final heading is free. The goal is a position only.
struct HybridRequest {
  Vec3 start = Vec3::Zero();
  double start_yaw = 0.0;
  Vec3 goal = Vec3::Zero();
  double goal_tolerance_m = 0.15;
  // Optional heading the body should have at the goal, within the tolerance either way.
  bool goal_heading_set = false;
  double goal_yaw = 0.0;
  double goal_heading_tolerance = M_PI / 4;
  double turn_radius_m = 1.0;
  // Total reverse travel allowed on the path: 0 for bodies that cannot reverse, the creep left
  // for bodies that cannot see behind.
  double reverse_limit_m = std::numeric_limits<double>::infinity();
  // Footprint probe points in the base frame. A pose is free when every probe is outside the
  // occupied cells; centre clearance beyond `outer_radius_m` skips the probes.
  const std::vector<Vec2>* probes = nullptr;
  double outer_radius_m = 0.0;
  double comfort_clearance_m = 0.0;
  double unknown_penalty = 1.0;
  int max_expansions = 30000;
};

struct HybridPathPoint {
  Vec3 position;
  double yaw = 0.0;
  bool reverse = false;  // the body drives backwards from the previous point to this one
};

struct HybridResult {
  bool reached = false;
  std::vector<HybridPathPoint> points;
  int expansions = 0;
};

class HybridSearch {
 public:
  HybridResult find(const grid::SearchGrid& grid, const HybridRequest& request);
  // Whether the body fits at every point (positions and yaws) on the current grid.
  bool path_free(const grid::SearchGrid& grid, const HybridRequest& request,
                 const std::vector<HybridPathPoint>& points) const;

 private:
  struct Node {
    double x, y, yaw;
    double g;
    double reverse_used;
    int parent;
    bool reverse;
    bool closed;
  };

  bool pose_free(const grid::SearchGrid& grid, const HybridRequest& req, double x, double y, double yaw) const;
  void heuristic_field(const grid::SearchGrid& grid, const HybridRequest& req, const grid::Index3& goal);
  double heuristic(const grid::SearchGrid& grid, double x, double y) const;
  // Turn-then-straight curves from `n` to the goal, forward and reverse, tightest turn first
  // on either side. Fills `out` (excluding `n` itself) with the cheapest free one.
  bool analytic(const grid::SearchGrid& grid, const HybridRequest& req, const Node& n,
                std::vector<HybridPathPoint>& out) const;

  std::vector<Node> nodes_;
  std::unordered_map<uint64_t, int> index_;
  std::vector<float> h_;
};

}  // namespace argus::node

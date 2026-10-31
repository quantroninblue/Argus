#pragma once

#include <limits>
#include <vector>

#include "argus/capabilities/capabilities.hpp"
#include "argus/capabilities/profile.hpp"
#include "argus/grid/search.hpp"
#include "argus/sitemap/site_map.hpp"

namespace argus::sitemap {

// What the route planner needs to know about a body, from its capabilities. Same clearances
// as Argo's local planner, so a route the hub finds is one Argo can follow.
struct RouteBody {
  bool planar = true;
  // Height band a planar body can hit, relative to its base origin.
  double z_low_m = 0.0;
  double z_high_m = 1.0;
  double min_clearance_m = 0.0;
  double comfort_clearance_m = 0.0;
  // Room a waypoint should have so the body can turn there: the footprint's circumradius for
  // planar bodies. Waypoints are put where the path has this much, when it does anywhere.
  double turn_clearance_m = 0.0;

  static RouteBody from(const capabilities::Profile& profile, const capabilities::Capabilities& caps,
                        double min_obstacle_height_m, double resolution_m);
};

struct RouteRequest {
  Vec3 start = Vec3::Zero();
  Vec3 goal = Vec3::Zero();
  // The search box covers the mapped area, start and goal, plus this much unknown space.
  double margin_m = 3.0;
  // Extra cost per metre through unknown space, on top of the distance itself.
  double unknown_penalty = 2.0;
  double aerial_resolution_m = 0.2;
  // Longest leg between waypoints, so each one is inside Argo's local view when it starts it.
  double max_leg_m = 3.0;
  int max_expansions = 2'000'000;
  // Aerial routes keep the body between these heights (site frame), e.g. a body radius above
  // the site's floor and below its ceiling. NaN means no limit beyond the search box.
  double z_min_m = std::numeric_limits<double>::quiet_NaN();
  double z_max_m = std::numeric_limits<double>::quiet_NaN();
  // Zones the exploration memory has ruled out, as (x, y, z, radius): discs for planar bodies,
  // balls for aerial ones. Treated as obstacles, except one that contains the start.
  std::vector<Eigen::Vector4d> blocked;
};

struct RouteWaypoint {
  Vec3 position = Vec3::Zero();
  // The leg ending here crosses cells nobody has observed.
  bool speculative = false;
};

struct Route {
  bool reached = false;
  // Not reached and nothing more to search: known obstacles and blocked zones cut the goal off.
  bool exhausted = false;
  // The goal cell itself is occupied, blocked or too close to an obstacle.
  bool goal_blocked = false;
  std::vector<Vec3> path;
  std::vector<RouteWaypoint> waypoints;
  double length_m = 0.0;
  double unknown_m = 0.0;  // part of the length through unobserved cells
};

// Plans through the site map in a body's configuration space. Prefers observed free space;
// routes through unknown space are allowed but cost more and are marked speculative. Output is
// a short list of waypoints, not a dense path; Argo plans the motion between them.
class RoutePlanner {
 public:
  explicit RoutePlanner(RouteBody body) : body_(body) {}

  Route plan(const SiteMap& map, const RouteRequest& request);

  const RouteBody& body() const { return body_; }
  // The grid of the last plan, for inspection and tests.
  const grid::SearchGrid& grid() const { return grid_; }

 private:
  void build_grid(const SiteMap& map, const RouteRequest& request);
  static void block(std::vector<Cell>& cells, const Index3& origin, const Index3& dims, double res,
                    bool planar, const RouteRequest& request);
  bool clear_line(const Vec3& a, const Vec3& b) const;
  bool unknown_on(const Vec3& a, const Vec3& b) const;

  RouteBody body_;
  grid::SearchGrid grid_;
  grid::PathSearch search_;
};

}  // namespace argus::sitemap

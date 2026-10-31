#include "argus/sitemap/route.hpp"

#include <algorithm>
#include <cmath>

namespace argus::sitemap {

namespace {

// Argo's local planner keeps the same comfort margin (cpp/node/src/planner.cpp).
constexpr double kComfortMargin = 0.3;
// Unknown space above and below the mapped volume that aerial routes may use.
constexpr double kAerialVerticalMargin = 1.0;
// Shortest leg worth a waypoint when moving one back to where the body can turn.
constexpr double kMinLeg = 0.5;

}  // namespace

RouteBody RouteBody::from(const capabilities::Profile& profile, const capabilities::Capabilities& caps,
                          double min_obstacle_height_m, double resolution_m) {
  RouteBody b;
  b.planar = caps.motion.space == capabilities::MotionSpace::Planar;
  b.z_low_m = profile.geometry.ground_z_m.value_or(0.0) + min_obstacle_height_m;
  b.z_high_m = caps.collision.aabb_max.z() + caps.collision.padding_m;
  if (b.planar) {
    b.min_clearance_m = std::max(0.0, caps.collision.inscribed_radius_m - 0.5 * resolution_m);
    for (const auto& v : caps.collision.footprint) {
      b.turn_clearance_m = std::max(b.turn_clearance_m, v.norm());
    }
  } else {
    b.min_clearance_m = caps.collision.bounding_radius_m;
    b.turn_clearance_m = b.min_clearance_m;
  }
  b.comfort_clearance_m = caps.collision.bounding_radius_m + kComfortMargin;
  return b;
}

void RoutePlanner::block(std::vector<Cell>& cells, const Index3& origin, const Index3& dims, double res,
                         bool planar, const RouteRequest& req) {
  for (const Eigen::Vector4d& zone : req.blocked) {
    const Vec3 c = zone.head<3>();
    const double r = zone[3];
    Vec3 offset = req.start - c;
    if (planar) offset.z() = 0.0;
    if (offset.norm() <= r) {
      continue;
    }
    const Index3 lo = grid::index_of(c - Vec3::Constant(r), res) - origin;
    const Index3 hi = grid::index_of(c + Vec3::Constant(r), res) - origin;
    for (int x = std::max(0, lo.x()); x <= std::min(dims.x() - 1, hi.x()); ++x) {
      for (int y = std::max(0, lo.y()); y <= std::min(dims.y() - 1, hi.y()); ++y) {
        const int z0 = planar ? 0 : std::max(0, lo.z());
        const int z1 = planar ? 0 : std::min(dims.z() - 1, hi.z());
        for (int z = z0; z <= z1; ++z) {
          Vec3 p = grid::center_of(origin + Index3(x, y, z), res) - c;
          if (planar) p.z() = 0.0;
          if (p.norm() <= r) {
            cells[(static_cast<size_t>(x) * dims.y() + y) * dims.z() + z] = Cell::Occupied;
          }
        }
      }
    }
  }
}

void RoutePlanner::build_grid(const SiteMap& map, const RouteRequest& req) {
  const double res = map.resolution();
  Vec3 lo = req.start.cwiseMin(req.goal);
  Vec3 hi = req.start.cwiseMax(req.goal);
  Index3 map_lo, map_hi;
  if (map.bounds(map_lo, map_hi)) {
    lo = lo.cwiseMin(map_lo.cast<double>() * res);
    hi = hi.cwiseMax(map_hi.cast<double>() * res);
  }
  if (body_.planar) {
    const Index3 a = grid::index_of(lo - Vec3::Constant(req.margin_m), res);
    const Index3 b = grid::index_of(hi + Vec3::Constant(req.margin_m), res) + Index3::Ones();
    const double z = req.start.z();
    PlanarView view = map.planar_view(z + body_.z_low_m, z + body_.z_high_m, a, b);
    const Index3 origin(a.x(), a.y(), 0);
    const Index3 dims(view.nx, view.ny, 1);
    block(view.cells, origin, dims, res, true, req);
    grid_.assign(origin, dims, res, true, z, std::move(view.cells));
    return;
  }
  const int factor = std::max(1, static_cast<int>(std::lround(req.aerial_resolution_m / res)));
  const double coarse = factor * res;
  const Vec3 margin(req.margin_m, req.margin_m, std::min(req.margin_m, kAerialVerticalMargin));
  Index3 a = grid::index_of(lo - margin, coarse);
  Index3 b = grid::index_of(hi + margin, coarse) + Index3::Ones();
  // Only cells whose centre is inside the height limits.
  if (std::isfinite(req.z_min_m)) {
    a.z() = std::max(a.z(), static_cast<int>(std::ceil(req.z_min_m / coarse - 0.5)));
  }
  if (std::isfinite(req.z_max_m)) {
    b.z() = std::min(b.z(), static_cast<int>(std::floor(req.z_max_m / coarse - 0.5)) + 1);
  }
  b.z() = std::max(b.z(), a.z() + 1);
  std::vector<Cell> cells = map.coarse_view(factor, a, b);
  block(cells, a, b - a, coarse, false, req);
  grid_.assign(a, b - a, coarse, false, 0.0, std::move(cells));
}

bool RoutePlanner::clear_line(const Vec3& a, const Vec3& b) const {
  const double step = 0.5 * grid_.resolution();
  const int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / step)));
  for (int k = 0; k <= n; ++k) {
    const Vec3 p = a + (b - a) * (static_cast<double>(k) / n);
    if (grid_.state_at(p) == Cell::Occupied || grid_.clearance_at(p) < body_.min_clearance_m) {
      return false;
    }
  }
  return true;
}

bool RoutePlanner::unknown_on(const Vec3& a, const Vec3& b) const {
  const double step = 0.5 * grid_.resolution();
  const int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / step)));
  for (int k = 0; k <= n; ++k) {
    if (grid_.state_at(a + (b - a) * (static_cast<double>(k) / n)) == Cell::Unknown) {
      return true;
    }
  }
  return false;
}

Route RoutePlanner::plan(const SiteMap& map, const RouteRequest& req) {
  build_grid(map, req);
  grid::PathRequest pr;
  pr.start = req.start;
  pr.goal = req.goal;
  pr.min_clearance_m = body_.min_clearance_m;
  pr.comfort_clearance_m = body_.comfort_clearance_m;
  pr.unknown_penalty = req.unknown_penalty;
  pr.max_expansions = req.max_expansions;
  const grid::PathResult found = search_.find(grid_, pr);

  Route route;
  route.reached = found.reached;
  route.exhausted = found.exhausted;
  route.goal_blocked = found.goal_blocked;
  route.path = found.points;
  const std::vector<Vec3>& p = route.path;
  for (size_t k = 0; k + 1 < p.size(); ++k) {
    const double len = (p[k + 1] - p[k]).norm();
    route.length_m += len;
    if (grid_.state_at(0.5 * (p[k] + p[k + 1])) == Cell::Unknown) {
      route.unknown_m += len;
    }
  }

  // Greedy shortening: from each waypoint, go as far along the path as a straight, clear leg
  // allows, without mixing observed and unobserved stretches in one leg. The first waypoint
  // into unknown space then sits near the edge of what has been seen. A waypoint is where the
  // body has to arrive and maybe turn, so it goes at the farthest point of the leg with room
  // to turn; only where the path never has that room does it go wherever the leg ends.
  size_t i = 0;
  while (i + 1 < p.size()) {
    const bool speculative = unknown_on(p[i], p[i + 1]);
    size_t best = i + 1;
    for (size_t j = i + 2; j < p.size(); ++j) {
      if ((p[j] - p[i]).norm() > req.max_leg_m || unknown_on(p[j - 1], p[j]) != speculative ||
          !clear_line(p[i], p[j])) {
        break;
      }
      best = j;
    }
    if (best + 1 < p.size() && grid_.clearance_at(p[best]) < body_.turn_clearance_m) {
      for (size_t j = best; j > i + 1 && (p[j] - p[i]).norm() >= kMinLeg; --j) {
        if (grid_.clearance_at(p[j]) >= body_.turn_clearance_m) {
          best = j;
          break;
        }
      }
    }
    route.waypoints.push_back({p[best], speculative || unknown_on(p[i], p[best])});
    i = best;
  }
  return route;
}

}  // namespace argus::sitemap

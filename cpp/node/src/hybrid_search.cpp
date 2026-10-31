#include "argus/node/hybrid_search.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

namespace argus::node {

using grid::Cell;
using grid::Index3;
using grid::SearchGrid;

namespace {

constexpr int kHeadings = 72;
constexpr double kBin = 2.0 * M_PI / kHeadings;
// Cost multipliers and the cost of a change of direction, in metres. Nav2's Smac Hybrid
// defaults are 1.2 for turning and 2.0 for reversing.
constexpr double kTurnPenalty = 1.2;
constexpr double kReversePenalty = 2.0;
constexpr double kCuspCost = 1.0;
// The direct curve to the goal is tried on every few expansions once the goal is this close.
constexpr double kAnalyticRange = 4.0;
constexpr int kAnalyticEvery = 4;

int heading_bin(double yaw) {
  const int b = static_cast<int>(std::lround(yaw / kBin)) % kHeadings;
  return b < 0 ? b + kHeadings : b;
}

// One arc of length `len` (signed curvature `k`, 0 for straight), forward or backward.
void advance(double& x, double& y, double& yaw, double len, double k, bool reverse) {
  const double d = reverse ? -len : len;
  if (std::abs(k) < 1e-12) {
    x += d * std::cos(yaw);
    y += d * std::sin(yaw);
    return;
  }
  const double next = yaw + d * k;
  x += (std::sin(next) - std::sin(yaw)) / k;
  y -= (std::cos(next) - std::cos(yaw)) / k;
  yaw = next;
}

}  // namespace

bool HybridSearch::pose_free(const SearchGrid& grid, const HybridRequest& req, double x, double y,
                             double yaw) const {
  const Vec3 c(x, y, req.start.z());
  if (!grid.contains(grid.local_of(c))) {
    return false;
  }
  if (grid.clearance_at(c) > req.outer_radius_m) {
    return true;
  }
  const double cs = std::cos(yaw);
  const double sn = std::sin(yaw);
  const double inside = 0.5 * grid.resolution();
  for (const Vec2& p : *req.probes) {
    const Vec3 w(x + cs * p.x() - sn * p.y(), y + sn * p.x() + cs * p.y(), req.start.z());
    if (grid.clearance_at(w) < inside) {
      return false;
    }
  }
  return true;
}

void HybridSearch::heuristic_field(const SearchGrid& grid, const HybridRequest& req, const Index3& goal) {
  // Shortest 8-connected distance to the goal through cells the body's centre can be in,
  // ignoring the turn radius: it never overestimates the arc path's cost.
  const Index3& dims = grid.dims();
  const size_t n = static_cast<size_t>(dims.x()) * dims.y();
  h_.assign(n, std::numeric_limits<float>::infinity());
  const double res = grid.resolution();
  auto lin = [&](int x, int y) { return static_cast<size_t>(x) * dims.y() + y; };
  using Entry = std::pair<float, size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  h_[lin(goal.x(), goal.y())] = 0.0f;
  open.emplace(0.0f, lin(goal.x(), goal.y()));
  while (!open.empty()) {
    const auto [d, k] = open.top();
    open.pop();
    if (d > h_[k]) {
      continue;
    }
    const int cx = static_cast<int>(k / dims.y());
    const int cy = static_cast<int>(k % dims.y());
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        const Index3 nb(cx + dx, cy + dy, 0);
        if (!grid.contains(nb) || grid.state(nb) == Cell::Occupied) continue;
        double step = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * res;
        if (grid.state(nb) == Cell::Unknown) step *= 1.0 + req.unknown_penalty;
        const size_t nk = lin(nb.x(), nb.y());
        const float nd = d + static_cast<float>(step);
        if (nd < h_[nk]) {
          h_[nk] = nd;
          open.emplace(nd, nk);
        }
      }
    }
  }
}

double HybridSearch::heuristic(const SearchGrid& grid, double x, double y) const {
  const Index3 i = grid.local_of(Vec3(x, y, 0.0));
  if (!grid.contains(Index3(i.x(), i.y(), 0))) {
    return std::numeric_limits<double>::infinity();
  }
  return h_[static_cast<size_t>(i.x()) * grid.dims().y() + i.y()];
}

bool HybridSearch::analytic(const SearchGrid& grid, const HybridRequest& req, const Node& n,
                            std::vector<HybridPathPoint>& out) const {
  const double r = req.turn_radius_m;
  const double step = 0.5 * grid.resolution();
  double best = std::numeric_limits<double>::infinity();
  for (int dir = 0; dir < 2; ++dir) {
    const bool reverse = dir == 1;
    if (reverse && req.reverse_limit_m - n.reverse_used <= 0.0) continue;
    // Driving backwards traces the same curves as driving forwards with the heading flipped.
    const double travel = reverse ? n.yaw + M_PI : n.yaw;
    for (int side : {1, -1}) {
      // Centre of the turning circle, and the arc to the point where the heading line meets
      // the goal (tangent from the goal to the circle).
      const double cx = n.x - side * r * std::sin(travel);
      const double cy = n.y + side * r * std::cos(travel);
      const double gx = req.goal.x() - cx;
      const double gy = req.goal.y() - cy;
      const double dist = std::hypot(gx, gy);
      if (dist < r) continue;
      const double phi = std::atan2(gy, gx);
      const double alpha = std::acos(r / dist);
      const double tangent = side > 0 ? phi - alpha : phi + alpha;
      const double start = side > 0 ? travel - M_PI_2 : travel + M_PI_2;
      double sweep = side > 0 ? tangent - start : start - tangent;
      sweep = std::fmod(std::fmod(sweep, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
      const double arc = r * sweep;
      const double line = std::sqrt(std::max(0.0, dist * dist - r * r));
      const double length = arc + line;
      if (reverse && n.reverse_used + length > req.reverse_limit_m) continue;
      const double cost = length * (reverse ? kReversePenalty : 1.0) + (reverse != n.reverse ? kCuspCost : 0.0);
      if (cost >= best) continue;

      std::vector<HybridPathPoint> pts;
      double x = n.x, y = n.y, yaw = travel;
      bool free = true;
      const int arc_steps = static_cast<int>(std::ceil(arc / step));
      const int line_steps = static_cast<int>(std::ceil(line / step));
      for (int i = 0; i < arc_steps + line_steps && free; ++i) {
        if (i < arc_steps) {
          advance(x, y, yaw, arc / arc_steps, side / r, false);
        } else {
          advance(x, y, yaw, line / line_steps, 0.0, false);
        }
        const double body_yaw = reverse ? yaw - M_PI : yaw;
        free = pose_free(grid, req, x, y, body_yaw);
        pts.push_back({Vec3(x, y, req.start.z()), wrap_angle(body_yaw), reverse});
      }
      if (free && req.goal_heading_set && !pts.empty() &&
          std::abs(wrap_angle(pts.back().yaw - req.goal_yaw)) > req.goal_heading_tolerance) {
        free = false;
      }
      if (free) {
        best = cost;
        out = std::move(pts);
      }
    }
  }
  return std::isfinite(best);
}

bool HybridSearch::path_free(const SearchGrid& grid, const HybridRequest& req,
                             const std::vector<HybridPathPoint>& points) const {
  for (const auto& p : points) {
    if (!pose_free(grid, req, p.position.x(), p.position.y(), p.yaw)) return false;
  }
  return true;
}

HybridResult HybridSearch::find(const SearchGrid& grid, const HybridRequest& req) {
  HybridResult result;
  nodes_.clear();
  index_.clear();
  const Index3 goal_i = grid.local_of(Vec3(req.goal.x(), req.goal.y(), req.start.z()));
  if (!grid.planar() || req.probes == nullptr || req.turn_radius_m <= 0.0 || !grid.contains(goal_i) ||
      !grid.contains(grid.local_of(req.start))) {
    return result;
  }
  heuristic_field(grid, req, goal_i);

  const double res = grid.resolution();
  // Long enough that the tightest turn changes heading by at least one bin.
  const double len = std::max(std::sqrt(2.0) * res, req.turn_radius_m * kBin * 1.05);
  const double k = 1.0 / req.turn_radius_m;
  const Index3& dims = grid.dims();
  auto key = [&](double x, double y, double yaw) {
    const Index3 i = grid.local_of(Vec3(x, y, 0.0));
    return (static_cast<uint64_t>(i.x()) * dims.y() + i.y()) * kHeadings + heading_bin(yaw);
  };

  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  nodes_.push_back({req.start.x(), req.start.y(), req.start_yaw, 0.0, 0.0, -1, false, false});
  index_[key(req.start.x(), req.start.y(), req.start_yaw)] = 0;
  open.emplace(heuristic(grid, req.start.x(), req.start.y()), 0);

  int found = -1;
  std::vector<HybridPathPoint> tail;
  while (!open.empty() && result.expansions < req.max_expansions) {
    const int cur = open.top().second;
    open.pop();
    if (nodes_[cur].closed) continue;
    nodes_[cur].closed = true;
    ++result.expansions;
    const Node n = nodes_[cur];
    if (std::hypot(n.x - req.goal.x(), n.y - req.goal.y()) <= req.goal_tolerance_m &&
        (!req.goal_heading_set || std::abs(wrap_angle(n.yaw - req.goal_yaw)) <= req.goal_heading_tolerance)) {
      found = cur;
      break;
    }
    const double h = heuristic(grid, n.x, n.y);
    if (h < kAnalyticRange && (result.expansions % kAnalyticEvery == 1 || cur == 0) && analytic(grid, req, n, tail)) {
      found = cur;
      break;
    }
    for (int dir = 0; dir < 2; ++dir) {
      const bool reverse = dir == 1;
      if (reverse && n.reverse_used + len > req.reverse_limit_m) continue;
      for (double curv : {0.0, k, -k}) {
        double x = n.x, y = n.y, yaw = n.yaw;
        bool free = true;
        // Checked at every half cell along the arc.
        const int sub = std::max(1, static_cast<int>(std::ceil(len / (0.5 * res))));
        for (int s = 0; s < sub && free; ++s) {
          advance(x, y, yaw, len / sub, curv, reverse);
          free = pose_free(grid, req, x, y, yaw);
        }
        if (!free) continue;
        const Vec3 p(x, y, req.start.z());
        const double hn = heuristic(grid, x, y);
        if (!std::isfinite(hn)) continue;
        double cost = len * (curv != 0.0 ? kTurnPenalty : 1.0) * (reverse ? kReversePenalty : 1.0);
        if (grid.state_at(p) == Cell::Unknown) cost *= 1.0 + req.unknown_penalty;
        const double clear = grid.clearance_at(p);
        if (req.comfort_clearance_m > 0.0 && clear < req.comfort_clearance_m) {
          cost += len * 2.0 * (req.comfort_clearance_m - clear) / req.comfort_clearance_m;
        }
        if (cur != 0 && reverse != n.reverse) cost += kCuspCost;
        const double g = n.g + cost;
        const uint64_t kk = key(x, y, yaw);
        const auto it = index_.find(kk);
        if (it != index_.end() && (nodes_[it->second].closed || nodes_[it->second].g <= g)) continue;
        const Node next{x, y, wrap_angle(yaw), g, n.reverse_used + (reverse ? len : 0.0), cur, reverse, false};
        int id;
        if (it == index_.end()) {
          id = static_cast<int>(nodes_.size());
          nodes_.push_back(next);
          index_[kk] = id;
        } else {
          id = it->second;
          nodes_[id] = next;
        }
        open.emplace(g + hn, id);
      }
    }
  }
  if (found < 0) {
    return result;
  }
  std::vector<HybridPathPoint> rev;
  for (int i = found; i >= 0; i = nodes_[i].parent) {
    rev.push_back({Vec3(nodes_[i].x, nodes_[i].y, req.start.z()), nodes_[i].yaw, nodes_[i].reverse});
  }
  result.points.assign(rev.rbegin(), rev.rend());
  result.points.front().position = req.start;
  result.points.insert(result.points.end(), tail.begin(), tail.end());
  result.reached = true;
  return result;
}

}  // namespace argus::node

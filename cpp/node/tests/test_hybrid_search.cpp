#include <gtest/gtest.h>

#include <cmath>

#include "argus/node/hybrid_search.hpp"

using argus::grid::Cell;
using argus::grid::Index3;
using argus::grid::SearchGrid;
using namespace argus::node;

namespace {

constexpr double kRes = 0.1;

// A free 10 x 10 m grid from (-5, -5), with `occupied(x, y)` cells blocked.
template <typename F>
SearchGrid make_grid(F occupied) {
  const int n = 100;
  std::vector<Cell> cells(n * n, Cell::Free);
  for (int x = 0; x < n; ++x) {
    for (int y = 0; y < n; ++y) {
      if (occupied(-5.0 + (x + 0.5) * kRes, -5.0 + (y + 0.5) * kRes)) {
        cells[static_cast<size_t>(x) * n + y] = Cell::Occupied;
      }
    }
  }
  SearchGrid g;
  g.assign(Index3(-50, -50, 0), Index3(n, n, 1), kRes, true, 0.0, std::move(cells));
  return g;
}

std::vector<Vec2> box_probes(double length, double width) {
  std::vector<Vec2> p;
  for (double x = -length / 2; x <= length / 2 + 1e-9; x += 0.05) {
    p.emplace_back(x, width / 2);
    p.emplace_back(x, -width / 2);
  }
  for (double y = -width / 2; y <= width / 2 + 1e-9; y += 0.05) {
    p.emplace_back(length / 2, y);
    p.emplace_back(-length / 2, y);
  }
  return p;
}

HybridRequest request(const std::vector<Vec2>& probes, double x, double y) {
  HybridRequest r;
  r.goal = Vec3(x, y, 0.0);
  r.turn_radius_m = 0.8;
  r.probes = &probes;
  r.outer_radius_m = 0.45;
  return r;
}

// Every step of the path turns no tighter than the radius allows, and the body never jumps.
void expect_drivable(const HybridResult& r, double radius) {
  for (size_t i = 1; i < r.points.size(); ++i) {
    const double ds = (r.points[i].position - r.points[i - 1].position).norm();
    EXPECT_LT(ds, 0.5) << i;
    if (ds > 1e-3) {
      const double dyaw = std::abs(wrap_angle(r.points[i].yaw - r.points[i - 1].yaw));
      EXPECT_LE(dyaw / ds, 1.0 / radius + 0.15) << i;
    }
  }
}

}  // namespace

TEST(HybridSearch, ReachesAGoalBesideTheBodyInsideItsTurningCircle) {
  const SearchGrid g = make_grid([](double, double) { return false; });
  const auto probes = box_probes(0.6, 0.4);
  HybridSearch search;
  HybridRequest req = request(probes, 0.0, -0.97);  // right beside it, as in the Gazebo run
  const HybridResult r = search.find(g, req);
  ASSERT_TRUE(r.reached);
  EXPECT_LT((r.points.back().position - req.goal).norm(), 0.2);
  expect_drivable(r, req.turn_radius_m);

  // Without reversing it still gets there, by a loop.
  req.reverse_limit_m = 0.0;
  const HybridResult forward = search.find(g, req);
  ASSERT_TRUE(forward.reached);
  for (const auto& p : forward.points) EXPECT_FALSE(p.reverse);
  expect_drivable(forward, req.turn_radius_m);
}

TEST(HybridSearch, BacksOutOfADeadEndCorridorItCannotTurnIn) {
  // A 0.9 m wide corridor along +x, closed at x = 1.5. The goal is behind the body.
  const SearchGrid g = make_grid([](double x, double y) {
    return (std::abs(y) > 0.45 && std::abs(y) < 0.7 && x > -1.5 && x < 1.6) || (x > 1.5 && x < 1.7 && std::abs(y) < 0.7);
  });
  const auto probes = box_probes(0.6, 0.4);
  HybridSearch search;
  HybridRequest req = request(probes, -3.0, 0.0);
  const HybridResult r = search.find(g, req);
  ASSERT_TRUE(r.reached);
  bool reversed = false;
  for (const auto& p : r.points) reversed = reversed || p.reverse;
  EXPECT_TRUE(reversed);
  expect_drivable(r, req.turn_radius_m);

  // A body that may not reverse has no way out.
  req.reverse_limit_m = 0.0;
  EXPECT_FALSE(search.find(g, req).reached);
}

TEST(HybridSearch, ReverseTravelStaysWithinTheLimit) {
  const SearchGrid g = make_grid([](double, double) { return false; });
  const auto probes = box_probes(0.6, 0.4);
  HybridSearch search;
  HybridRequest req = request(probes, -2.0, 0.0);  // straight behind
  req.reverse_limit_m = 0.5;
  const HybridResult r = search.find(g, req);
  ASSERT_TRUE(r.reached);
  double reverse = 0.0;
  for (size_t i = 1; i < r.points.size(); ++i) {
    if (r.points[i].reverse) reverse += (r.points[i].position - r.points[i - 1].position).norm();
  }
  EXPECT_LE(reverse, 0.5 + 1e-6);
}

TEST(HybridSearch, ArrivesLinedUpWithTheNextLegWhenAsked) {
  const SearchGrid g = make_grid([](double, double) { return false; });
  const auto probes = box_probes(0.6, 0.4);
  HybridSearch search;
  HybridRequest req = request(probes, 2.5, 1.0);
  req.reverse_limit_m = 0.0;
  req.goal_heading_set = true;
  req.goal_yaw = -M_PI / 2;  // the next leg heads south
  const HybridResult r = search.find(g, req);
  ASSERT_TRUE(r.reached);
  EXPECT_LE(std::abs(wrap_angle(r.points.back().yaw - req.goal_yaw)), req.goal_heading_tolerance + 1e-9);
  EXPECT_LT((r.points.back().position - req.goal).norm(), 0.2);
  expect_drivable(r, req.turn_radius_m);
}

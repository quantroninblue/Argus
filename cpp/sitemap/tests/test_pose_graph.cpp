#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "argus/sitemap/pose_graph.hpp"

using namespace argus::sitemap;

namespace {

constexpr double kQuarter = 1.5707963267948966;

// Four sides of a square, one step of `side` metres per edge, turning left at each corner, as
// odometry that overturns by `yaw_error` per step reports it.
std::vector<PoseNode> driven_square(int sides, double yaw_error) {
  std::vector<PoseNode> nodes(sides + 1);
  nodes[0].fixed = true;
  for (int i = 1; i <= sides; ++i) {
    const PoseNode& p = nodes[i - 1];
    nodes[i].x = p.x + 4.0 * std::cos(p.yaw);
    nodes[i].y = p.y + 4.0 * std::sin(p.yaw);
    nodes[i].yaw = p.yaw + kQuarter + yaw_error;
  }
  return nodes;
}

std::vector<PoseEdge> odometry(int sides, double yaw_error) {
  std::vector<PoseEdge> edges;
  for (int i = 0; i < sides; ++i) {
    PoseEdge e;
    e.from = i;
    e.to = i + 1;
    e.dx = 4.0;
    e.dyaw = kQuarter + yaw_error;
    e.sigma_m = 0.2;
    e.sigma_rad = 0.05;
    edges.push_back(e);
  }
  return edges;
}

PoseEdge closing(int from, int to) {
  PoseEdge e;
  e.from = from;
  e.to = to;
  e.sigma_m = 0.05;
  e.sigma_rad = 0.01;
  e.robust = true;
  return e;
}

}  // namespace

TEST(PoseGraph, ALoopEdgeBringsTheEndBackToTheStart) {
  const auto nodes = driven_square(4, 0.05);
  auto edges = odometry(4, 0.05);
  const double before = std::hypot(nodes[4].x, nodes[4].y);
  ASSERT_GT(before, 0.3);
  edges.push_back(closing(0, 4));
  edges.back().dyaw = 0.0;
  const auto r = optimize_pose_graph(nodes, edges);
  EXPECT_TRUE(r.converged);
  EXPECT_LT(std::hypot(r.nodes[4].x, r.nodes[4].y), 0.1);
  EXPECT_GT(r.weights.back(), 0.9);
  // The fixed node stays where it was.
  EXPECT_EQ(r.nodes[0].x, 0.0);
  EXPECT_EQ(r.nodes[0].y, 0.0);
}

TEST(PoseGraph, AConsistentGraphDoesNotMove) {
  const auto nodes = driven_square(4, 0.0);
  const auto r = optimize_pose_graph(nodes, odometry(4, 0.0));
  for (size_t i = 0; i < nodes.size(); ++i) {
    EXPECT_NEAR(r.nodes[i].x, nodes[i].x, 1e-6);
    EXPECT_NEAR(r.nodes[i].y, nodes[i].y, 1e-6);
  }
}

TEST(PoseGraph, ALoopEdgeThatDisagreesWithEverythingIsSwitchedOff) {
  const auto nodes = driven_square(8, 0.0);
  auto edges = odometry(8, 0.0);
  // Good edges from revisiting the corners, and one false match five metres off.
  for (int i = 0; i < 4; ++i) {
    edges.push_back(closing(i, i + 4));
  }
  PoseEdge wrong = closing(1, 6);
  wrong.dx = 5.0;
  edges.push_back(wrong);
  const auto r = optimize_pose_graph(nodes, edges);
  EXPECT_LT(r.weights.back(), 0.1);
  for (size_t i = 0; i < nodes.size(); ++i) {
    EXPECT_NEAR(r.nodes[i].x, nodes[i].x, 0.15);
    EXPECT_NEAR(r.nodes[i].y, nodes[i].y, 0.15);
  }
}

TEST(PoseGraph, SolvesAcrossTheAngleWrap) {
  std::vector<PoseNode> nodes(2);
  nodes[0].fixed = true;
  nodes[0].yaw = 3.1;
  nodes[1].yaw = -3.1;
  PoseEdge e;
  e.from = 0;
  e.to = 1;
  e.dyaw = 0.2;
  const auto r = optimize_pose_graph(nodes, {e});
  EXPECT_NEAR(std::remainder(r.nodes[1].yaw - 3.3, 2.0 * 3.14159265358979323846), 0.0, 1e-6);
}

TEST(PoseGraph, RefusesWhatItCannotSolve) {
  std::vector<PoseNode> free_nodes(2);
  EXPECT_THROW(optimize_pose_graph(free_nodes, {}), std::invalid_argument);
  auto nodes = free_nodes;
  nodes[0].fixed = true;
  PoseEdge outside;
  outside.to = 5;
  EXPECT_THROW(optimize_pose_graph(nodes, {outside}), std::invalid_argument);
  PoseEdge flat;
  flat.to = 1;
  flat.sigma_m = 0.0;
  EXPECT_THROW(optimize_pose_graph(nodes, {flat}), std::invalid_argument);
}

TEST(PoseGraph, AThousandNodesSolveQuickly) {
  const int n = 1000;
  auto nodes = driven_square(n, 0.01);
  auto edges = odometry(n, 0.01);
  for (int i = 0; i + 4 <= n; i += 4) {
    edges.push_back(closing(i, i + 4));
  }
  const auto r = optimize_pose_graph(nodes, edges);
  EXPECT_LT(std::hypot(r.nodes[n].x, r.nodes[n].y), 1.0);
}

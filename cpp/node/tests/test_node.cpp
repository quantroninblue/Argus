#include <gtest/gtest.h>

#include <cmath>

#include "argus/node/node.hpp"

using namespace argus::node;
namespace caps = argus::capabilities;

namespace {

caps::Profile example(const std::string& name) {
  return caps::load_profile_file(std::string(ARGUS_EXAMPLES_DIR) + "/" + name + ".yaml");
}

Odometry at(double stamp) {
  Odometry o;
  o.stamp = stamp;
  return o;
}

}  // namespace

TEST(Node, RayPatternsMatchSensorFields) {
  Node node(example("planar_differential"));
  EXPECT_EQ(node.range_sensor_ids(), (std::vector<std::string>{"front_depth", "front_lidar"}));
  const RayPattern& lidar = node.ray_pattern("front_lidar");
  EXPECT_EQ(lidar.rows, 1);
  EXPECT_EQ(lidar.cols, 270);
  for (const Vec3& d : lidar.directions) {
    EXPECT_NEAR(d.norm(), 1.0, 1e-12);
    EXPECT_NEAR(d.z(), 0.0, 1e-12);
    EXPECT_LE(std::abs(std::atan2(d.y(), d.x())), 135.0 * M_PI / 180.0);
  }
}

TEST(Node, RaysThatHitTheBodyAreKnown) {
  caps::Profile p = example("planar_differential");
  // The mast is behind the lidar, outside its 270 degree field of view.
  const Node plain(p);
  for (double h : plain.self_hits("front_lidar")) {
    EXPECT_FALSE(std::isfinite(h));
  }
  caps::Shape bumper;
  bumper.type = caps::ShapeType::Sphere;
  bumper.radius = 0.05;
  bumper.pose.xyz = Vec3(0.6, 0.0, 0.33);
  p.geometry.shapes.push_back(bumper);
  Node node(p);
  const auto& hits = node.self_hits("front_lidar");
  const RayPattern& lidar = node.ray_pattern("front_lidar");
  int blocked = 0;
  for (size_t i = 0; i < hits.size(); ++i) {
    if (std::isfinite(hits[i])) {
      ++blocked;
      EXPECT_GT(lidar.directions[i].x(), 0.99);
      EXPECT_GE(hits[i], 0.30 - 1e-9);
      EXPECT_LE(hits[i], 0.35);
    }
  }
  EXPECT_GE(blocked, 2);
}

TEST(Node, WaypointInterfaceIsRefused) {
  caps::Profile p = example("planar_differential");
  p.control.interface = caps::ControlInterface::Waypoint;
  EXPECT_THROW(Node node(p), std::invalid_argument);
}

TEST(Node, WithoutOdometryItStops) {
  Node node(example("planar_differential"));
  const TickResult r = node.tick(1.0);
  EXPECT_TRUE(r.command.stop);
  EXPECT_TRUE(r.safety_flags & kLocalization);
}

TEST(Node, PlansMustBeNewerAndUnexpired) {
  Node node(example("planar_differential"));
  node.on_odometry(at(1.0));
  Plan p;
  p.generation = 2;
  p.expires = 100.0;
  p.waypoints.push_back({"a", Vec3(3, 0, 0), 0.2, 0.0});
  EXPECT_TRUE(node.set_plan(p, 1.0));
  p.generation = 1;
  EXPECT_FALSE(node.set_plan(p, 1.0));
  p.generation = 3;
  p.expires = 0.5;
  EXPECT_FALSE(node.set_plan(p, 1.0));
  const TickResult r = node.tick(1.0);
  ASSERT_EQ(r.events.size(), 3u);
  EXPECT_EQ(r.events[0].kind, EventKind::PlanAccepted);
  EXPECT_EQ(r.events[1].kind, EventKind::PlanRejected);
  EXPECT_EQ(r.events[2].kind, EventKind::PlanRejected);
  EXPECT_EQ(r.target_id, "a");
}

TEST(Node, ExpiredPlanStopsTheRobot) {
  Node node(example("planar_differential"));
  node.on_odometry(at(1.0));
  Plan p;
  p.generation = 1;
  p.expires = 2.0;
  p.waypoints.push_back({"a", Vec3(3, 0, 0), 0.2, 0.0});
  ASSERT_TRUE(node.set_plan(p, 1.0));
  node.on_odometry(at(2.5));
  const TickResult r = node.tick(2.5);
  EXPECT_TRUE(r.command.stop);
  EXPECT_TRUE(r.safety_flags & kNoTarget);
  bool expired = false;
  for (const Event& e : r.events) expired = expired || e.kind == EventKind::PlanExpired;
  EXPECT_TRUE(expired);
}

TEST(Node, ReachedWaypointsAdvanceThePlan) {
  Node node(example("planar_differential"));
  node.on_odometry(at(1.0));
  Plan p;
  p.generation = 1;
  p.expires = 100.0;
  p.waypoints.push_back({"here", Vec3(0.1, 0, 0), 0.2, 0.0});
  p.waypoints.push_back({"there", Vec3(3, 0, 0), 0.2, 0.0});
  ASSERT_TRUE(node.set_plan(p, 1.0));
  const TickResult r = node.tick(1.0);
  EXPECT_EQ(r.target_id, "there");
}

TEST(Node, PoseSetpointStopHoldsWhereTheBodyWillRest) {
  Node node(example("aerial_multirotor"));
  Odometry o = at(1.0);
  o.pose.position = Vec3(0, 0, 1.5);
  o.twist.linear.x() = 1.5;
  node.on_odometry(o);
  const TickResult r = node.tick(1.0);
  EXPECT_TRUE(r.command.stop);
  EXPECT_NEAR(r.command.setpoint.position.x(), 1.5 * 1.5 / (2 * 3.0), 1e-9);
  EXPECT_NEAR(r.command.setpoint.position.z(), 1.5, 1e-9);
}

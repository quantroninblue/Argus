#include <gtest/gtest.h>

#include <cmath>

#include "argus/capabilities/capabilities.hpp"
#include "argus/node/safety.hpp"

using namespace argus::node;
namespace caps = argus::capabilities;

namespace {

// The example differential base, with a map built from a ring of rays at scan height that
// hit a wall `wall_x` metres ahead and nothing anywhere else.
struct Fixture {
  explicit Fixture(double wall_x)
      : profile(caps::load_profile_file(std::string(ARGUS_EXAMPLES_DIR) + "/planar_differential.yaml")),
        capabilities(caps::derive_capabilities(profile)),
        body(capabilities, 0.0, 0.05, 0.2),
        motion(capabilities.motion),
        map(LocalMapConfig{0.1, 6.0, 0.3, 1.2}) {
    map.recenter(Vec3::Zero());
    const Vec3 origin(0.25, 0.0, 0.33);
    std::vector<Vec3> ends;
    std::vector<bool> hits;
    for (int i = 0; i < 720; ++i) {
      const double a = i * M_PI / 360.0;
      const Vec3 dir(std::cos(a), std::sin(a), 0.0);
      double t = 5.0;
      if (dir.x() > 1e-6) {
        t = std::min(t, (wall_x - origin.x()) / dir.x());
      }
      ends.push_back(origin + t * dir);
      hits.push_back(t < 5.0);
    }
    for (int k = 0; k < 3; ++k) {
      map.integrate(origin, ends, hits);
    }
    for (const auto& s : profile.sensors) {
      clock.max_age[s.id] = s.max_age_s;
      clock.last_frame[s.id] = 10.0;
    }
    odom.stamp = 10.0;
  }

  SafetySupervisor supervisor() {
    return SafetySupervisor(capabilities, body, motion, SafetyConfig{}, 0.02, 0.1, clock.max_age);
  }
  SafetyInputs inputs(double now = 10.0) { return SafetyInputs{odom, map, clock, now}; }

  caps::Profile profile;
  caps::Capabilities capabilities;
  BodyVolume body;
  MotionModel motion;
  LocalMap map;
  SensorClock clock;
  Odometry odom;
};

Twist forward(double v) {
  Twist t;
  t.linear.x() = v;
  return t;
}

}  // namespace

TEST(Safety, OpenSpaceAhead) {
  Fixture f(4.0);
  SafetySupervisor s = f.supervisor();
  EXPECT_TRUE(s.admits(forward(1.0), f.inputs()));
  const auto v = s.check(forward(0.01), 10.0, f.inputs());
  EXPECT_EQ(v.flags, 0u);
  EXPECT_DOUBLE_EQ(v.velocity.linear.x(), 0.01);
}

TEST(Safety, AccelerationIsLimited) {
  Fixture f(4.0);
  SafetySupervisor s = f.supervisor();
  const auto v = s.check(forward(1.0), 10.0, f.inputs());
  EXPECT_TRUE(v.flags & kClamped);
  EXPECT_NEAR(v.velocity.linear.x(), 0.8 * 0.02, 1e-9);
}

TEST(Safety, WallInsideStoppingEnvelopeBlocks) {
  Fixture f(0.6);
  SafetySupervisor s = f.supervisor();
  EXPECT_FALSE(s.admits(forward(1.0), f.inputs()));
  EXPECT_TRUE(s.admits(Twist{}, f.inputs()));
}

TEST(Safety, CloseWallSlowsInsteadOfStopping) {
  // Braking from 1 m/s takes about 0.52 m; the padded front edge is 0.35 m ahead of the base.
  Fixture f(0.85);
  SafetySupervisor s = f.supervisor();
  EXPECT_FALSE(s.admits(forward(1.0), f.inputs()));
  EXPECT_TRUE(s.admits(forward(0.25), f.inputs()));
}

TEST(Safety, StaleSensorsStopTranslation) {
  Fixture f(4.0);
  SafetySupervisor s = f.supervisor();
  f.odom.stamp = 10.5;
  const auto v = s.check(forward(0.01), 10.5, f.inputs(10.5));
  EXPECT_EQ(v.flags, static_cast<uint32_t>(kStaleSensor));
  EXPECT_TRUE(v.velocity.is_zero());
}

TEST(Safety, OldInputsMissTheDeadline) {
  Fixture f(4.0);
  SafetySupervisor s = f.supervisor();
  f.odom.stamp = 11.0;
  f.clock.last_frame["front_lidar"] = 11.0;
  const auto v = s.check(forward(0.01), 10.0, f.inputs(11.0));
  EXPECT_TRUE(v.flags & kDeadline);
  EXPECT_TRUE(v.velocity.is_zero());
}

TEST(Safety, BadLocalizationStops) {
  Fixture f(4.0);
  SafetySupervisor s = f.supervisor();
  f.odom.healthy = false;
  const auto v = s.check(forward(0.01), 10.0, f.inputs());
  EXPECT_TRUE(v.flags & kLocalization);
  EXPECT_TRUE(v.velocity.is_zero());
}

TEST(Safety, BlindRearForbidsReversing) {
  Fixture f(4.0);
  SafetySupervisor s = f.supervisor();
  EXPECT_EQ(s.coverage(Vec3(-1, 0, 0), f.inputs()), Coverage::Uncovered);
  const auto v = s.check(forward(-0.01), 10.0, f.inputs());
  EXPECT_TRUE(v.flags & kUncovered);
  EXPECT_TRUE(v.velocity.is_zero());
}

TEST(Safety, CreepIsCappedWhenTheProfileAllowsIt) {
  Fixture f(4.0);
  f.capabilities.failsafe.uncovered_motion = caps::UncoveredMotion::Creep;
  f.capabilities.failsafe.creep_speed_mps = 0.05;
  SafetySupervisor s = f.supervisor();
  Twist v = forward(-0.3);
  for (int i = 0; i < 100; ++i) {
    v = s.check(forward(-0.3), 10.0, f.inputs()).velocity;
  }
  EXPECT_NEAR(v.linear.x(), -0.05, 1e-9);
}

TEST(Safety, UnreviewedProfileNeverMoves) {
  Fixture f(4.0);
  f.capabilities.safety_ready = false;
  SafetySupervisor s = f.supervisor();
  const auto v = s.check(forward(0.01), 10.0, f.inputs());
  EXPECT_TRUE(v.flags & kNotSafetyReady);
  EXPECT_TRUE(v.velocity.is_zero());
}

namespace {

TrackedObstacle person(double x, double y, double vx = 0.0, double vy = 0.0, bool has_moved = true) {
  TrackedObstacle o;
  o.id = 1;
  o.position = Eigen::Vector2d(x, y);
  o.velocity = Eigen::Vector2d(vx, vy);
  o.radius_m = 0.25;
  o.z_low_m = 0.0;
  o.z_high_m = 1.75;
  o.has_moved = has_moved;
  o.moving = vx != 0.0 || vy != 0.0;
  return o;
}

}  // namespace

TEST(Safety, SlowsNearSomethingThatHasMovedButNotNearOtherObstacles) {
  Fixture f(5.0);
  SafetySupervisor s = f.supervisor();
  // Standing 1 m to the side: well out of the way, but close.
  std::vector<TrackedObstacle> movers{person(0.0, 1.0)};
  SafetyInputs in = f.inputs();
  in.movers = &movers;
  uint32_t flags = 0;
  const Twist out = s.preview(forward(1.0), in, flags);
  EXPECT_TRUE(flags & kNearPeople);
  EXPECT_LT(out.linear.x(), 0.6);
  EXPECT_GE(out.linear.x(), SafetyConfig{}.people_speed_mps - 1e-9);
  // The same thing never seen moving is an ordinary obstacle: no slowdown.
  movers = {person(0.0, 1.0, 0.0, 0.0, false)};
  flags = 0;
  EXPECT_DOUBLE_EQ(s.preview(forward(1.0), in, flags).linear.x(), 1.0);
  EXPECT_FALSE(flags & kNearPeople);
}

TEST(Safety, MovingAwayFromSomeoneAlreadyCloseIsAllowedTowardThemIsNot) {
  Fixture f(5.0);
  SafetySupervisor s = f.supervisor();
  // Standing just behind the robot, inside the reach the supervisor keeps.
  std::vector<TrackedObstacle> movers{person(-0.6, 0.0)};
  SafetyInputs in = f.inputs();
  in.movers = &movers;
  uint32_t flags = 0;
  EXPECT_GT(s.preview(forward(0.3), in, flags).linear.x(), 0.0);
  EXPECT_FALSE(flags & kBlocked);
  // Ahead instead: driving at them is refused.
  movers = {person(0.6, 0.0)};
  flags = 0;
  EXPECT_DOUBLE_EQ(s.preview(forward(0.3), in, flags).linear.x(), 0.0);
  EXPECT_TRUE(flags & kBlocked);
}

TEST(Safety, SomeoneWalkingAtTheRobotBlocksDrivingTowardThem) {
  Fixture f(5.0);
  SafetySupervisor s = f.supervisor();
  // 1.6 m ahead at 1.4 m/s toward it: driving on meets them before the robot can stop.
  std::vector<TrackedObstacle> movers{person(1.6, 0.0, -1.4, 0.0)};
  SafetyInputs in = f.inputs();
  in.movers = &movers;
  uint32_t flags = 0;
  EXPECT_LT(s.preview(forward(0.5), in, flags).linear.x(), 0.5);
  EXPECT_TRUE(flags & (kBlocked | kSlowed));
}

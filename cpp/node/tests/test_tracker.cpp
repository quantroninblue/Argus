#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "argus/node/tracker.hpp"

using argus::node::ObstacleTracker;
using argus::node::TrackedObstacle;
using argus::node::Vec3;

namespace {

// The returns a lidar gets off a 0.5 m wide body centred at (x, y): its near face.
std::vector<Vec3> body_at(double x, double y) {
  std::vector<Vec3> out;
  for (double dy = -0.25; dy <= 0.25 + 1e-9; dy += 0.05) {
    out.emplace_back(x - 0.25, y + dy, 0.3);
  }
  return out;
}

}  // namespace

TEST(Tracker, FollowsAWalkerAndEstimatesItsVelocity) {
  ObstacleTracker t;
  for (int k = 0; k <= 15; ++k) {  // 1 s at 15 Hz, walking -x at 1.4 m/s
    const double s = k / 15.0;
    t.update(s, body_at(5.0 - 1.4 * s, 0.0));
  }
  const auto tracks = t.tracks(1.0);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_TRUE(tracks[0].moving);
  EXPECT_NEAR(tracks[0].velocity.x(), -1.4, 0.15);
  EXPECT_NEAR(tracks[0].velocity.y(), 0.0, 0.15);
  // Predicted ahead from its last sighting.
  EXPECT_NEAR(t.tracks(1.5)[0].position.x(), 5.0 - 0.25 - 1.4 * 1.5, 0.2);
}

TEST(Tracker, SomethingStandingStillIsNotMovingAndExplainsNothing) {
  ObstacleTracker t;
  for (int k = 0; k <= 15; ++k) {
    t.update(k / 15.0, body_at(3.0, 1.0));
  }
  EXPECT_TRUE(t.tracks(1.0).empty());          // no moving tracks
  EXPECT_EQ(t.tracks(1.0, false).size(), 1u);  // but it is tracked
  EXPECT_FALSE(t.explains(Vec3(2.75, 1.0, 0.3), 1.0));  // so its returns go into the map
}

TEST(Tracker, OnlyAMovingTrackExplainsReturnsNearIt) {
  ObstacleTracker t;
  for (int k = 0; k <= 15; ++k) {
    const double s = k / 15.0;
    t.update(s, body_at(1.0, -3.0 + 1.0 * s));  // walking +y at 1 m/s
  }
  EXPECT_TRUE(t.explains(Vec3(0.75, -2.0, 0.3), 1.0));
  EXPECT_FALSE(t.explains(Vec3(0.75, 1.0, 0.3), 1.0));  // far from it
}

TEST(Tracker, StructureTooLargeToBeAWalkerIsIgnored) {
  ObstacleTracker t;
  std::vector<Vec3> wall;
  for (double y = -2.0; y <= 2.0; y += 0.05) {
    wall.emplace_back(4.0, y, 0.3);
  }
  for (int k = 0; k <= 15; ++k) {
    t.update(k / 15.0, wall);
  }
  EXPECT_TRUE(t.tracks(1.0, false).empty());
}

TEST(Tracker, ATrackUnseenForItsLifetimeIsDroppedAndReported) {
  ObstacleTracker t;
  for (int k = 0; k <= 15; ++k) {
    const double s = k / 15.0;
    t.update(s, body_at(5.0 - 1.0 * s, 0.0));
  }
  t.update(1.5, {});
  EXPECT_EQ(t.tracks(1.5).size(), 1u);  // half a second unseen: still predicted
  t.update(2.2, {});
  EXPECT_TRUE(t.tracks(2.2, false).empty());
  const auto lost = t.take_lost();
  ASSERT_EQ(lost.size(), 1u);
  EXPECT_NEAR(lost[0].position.x(), 5.0 - 0.25 - 1.0 * 2.2, 0.25);  // where it was heading
  EXPECT_TRUE(t.take_lost().empty());
}

TEST(Tracker, TwoWalkersPassingKeepTheirOwnTracks) {
  ObstacleTracker t;
  for (int k = 0; k <= 15; ++k) {
    const double s = k / 15.0;
    std::vector<Vec3> both = body_at(5.0 - 1.0 * s, 1.5);
    const auto other = body_at(0.0 + 1.0 * s, -1.5);
    both.insert(both.end(), other.begin(), other.end());
    t.update(s, both);
  }
  const auto tracks = t.tracks(1.0);
  ASSERT_EQ(tracks.size(), 2u);
  for (const TrackedObstacle& o : tracks) {
    EXPECT_NEAR(std::abs(o.velocity.x()), 1.0, 0.15);
    EXPECT_GT(o.velocity.x() * (o.position.y() > 0 ? -1.0 : 1.0), 0.0);  // each its own way
  }
}

TEST(Tracker, StructureComingIntoViewIsNotMoving) {
  // The top of a low box seen from afar: more of it shows each frame, so its centre drifts at
  // walking speed, but its near edge never moves.
  ObstacleTracker t;
  for (int k = 0; k <= 15; ++k) {
    const double s = k / 15.0;
    std::vector<Vec3> returns;
    for (double y = 0.0; y <= 0.1 + 1.0 * s + 1e-9; y += 0.05) {
      returns.emplace_back(4.0, y, 0.2);
    }
    t.update(s, returns);
  }
  const auto tracks = t.tracks(1.0, false);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_GT(tracks[0].velocity.norm(), 0.3);
  EXPECT_FALSE(tracks[0].moving);
}

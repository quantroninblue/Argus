#include <cmath>

#include <gtest/gtest.h>

#include "argus/node/local_map.hpp"

using namespace argus::node;

namespace {

LocalMap small_map() {
  LocalMapConfig c;
  c.resolution_m = 0.1;
  c.half_width_m = 2.0;
  c.z_below_m = 0.5;
  c.z_above_m = 0.5;
  LocalMap map(c);
  map.recenter(Vec3::Zero());
  return map;
}

}  // namespace

TEST(LocalMap, RayClearsPathAndMarksHit) {
  LocalMap map = small_map();
  map.integrate(Vec3(0.05, 0.05, 0.05), {Vec3(1.05, 0.05, 0.05)}, {true});
  EXPECT_EQ(map.at(Vec3(0.55, 0.05, 0.05)), Cell::Free);
  EXPECT_EQ(map.at(Vec3(1.05, 0.05, 0.05)), Cell::Occupied);
  EXPECT_EQ(map.at(Vec3(1.25, 0.05, 0.05)), Cell::Unknown);
  EXPECT_EQ(map.at(Vec3(0.55, 0.55, 0.05)), Cell::Unknown);
}

TEST(LocalMap, RayWithoutReturnClearsItsEnd) {
  LocalMap map = small_map();
  map.integrate(Vec3(0.05, 0.05, 0.05), {Vec3(1.05, 0.05, 0.05)}, {false});
  EXPECT_EQ(map.at(Vec3(1.05, 0.05, 0.05)), Cell::Free);
}

TEST(LocalMap, GrazingRayInSameFrameDoesNotEraseHit) {
  LocalMap map = small_map();
  const Vec3 o(0.05, 0.05, 0.05);
  map.integrate(o, {Vec3(1.05, 0.05, 0.05), Vec3(1.95, 0.08, 0.05)}, {true, true});
  EXPECT_EQ(map.at(Vec3(1.05, 0.05, 0.05)), Cell::Occupied);
}

TEST(LocalMap, RepeatedMissesClearAnOldHit) {
  LocalMap map = small_map();
  const Vec3 o(0.05, 0.05, 0.05);
  map.integrate(o, {Vec3(1.05, 0.05, 0.05)}, {true});
  for (int i = 0; i < 10; ++i) {
    map.integrate(o, {Vec3(1.85, 0.05, 0.05)}, {true});
  }
  EXPECT_EQ(map.at(Vec3(1.05, 0.05, 0.05)), Cell::Free);
}

TEST(LocalMap, RecenterKeepsWhatStaysAndForgetsWhatLeaves) {
  LocalMap map = small_map();
  map.integrate(Vec3(0.05, 0.05, 0.05), {Vec3(1.05, 0.05, 0.05)}, {true});
  map.integrate(Vec3(0.05, 0.05, 0.05), {Vec3(-1.45, 0.05, 0.05)}, {true});
  map.recenter(Vec3(1.0, 0.0, 0.0));
  EXPECT_EQ(map.at(Vec3(1.05, 0.05, 0.05)), Cell::Occupied);
  EXPECT_EQ(map.at(Vec3(-1.45, 0.05, 0.05)), Cell::Unknown);
  // The slot the forgotten cell used now holds a cell on the far side, which must start unknown.
  EXPECT_EQ(map.at(Vec3(2.55, 0.05, 0.05)), Cell::Unknown);
  map.recenter(Vec3(50.0, 0.0, 0.0));
  EXPECT_EQ(map.at(Vec3(50.05, 0.05, 0.05)), Cell::Unknown);
}

TEST(LocalMap, OutsideTheWindowIsUnknownAndIgnored) {
  LocalMap map = small_map();
  map.integrate(Vec3(0.05, 0.05, 0.05), {Vec3(9.0, 0.05, 0.05)}, {true});
  EXPECT_EQ(map.at(Vec3(1.95, 0.05, 0.05)), Cell::Free);
  EXPECT_EQ(map.at(Vec3(9.0, 0.05, 0.05)), Cell::Unknown);
}

namespace {

// A fan of rays in one horizontal plane from the origin, all ending without a return at
// `range` except the ones in [hit_lo, hit_hi] of bearing, which hit at `hit_range`.
void scan(LocalMap& map, double range, double hit_lo = 1.0, double hit_hi = -1.0, double hit_range = 0.0) {
  const Vec3 o(0.05, 0.05, 0.05);
  std::vector<Vec3> ends;
  std::vector<bool> hits;
  for (double b = -0.6; b <= 0.6 + 1e-9; b += 0.01) {
    const bool hit = b >= hit_lo && b <= hit_hi;
    const double r = hit ? hit_range : range;
    ends.push_back(o + Vec3(r * std::cos(b), r * std::sin(b), 0.0));
    hits.push_back(hit);
  }
  map.integrate(o, ends, hits);
}

}  // namespace

TEST(LocalMap, OneScanPlaneMakesEverFreeSpace) {
  // The cells above and below a single scan plane are never observed; where no sensor can
  // see them they must not keep the plane's own cells from becoming ever-free.
  LocalMap map = small_map();
  for (int i = 0; i < 6; ++i) {
    scan(map, 1.8);
  }
  EXPECT_FALSE(map.ever_free(map.index_of(Vec3(1.05, 0.05, 0.05))));  // could be inside something
  const int plane = map.index_of(Vec3(0.0, 0.0, 0.05)).z();
  map.set_vertically_unseen([plane](const Index3& n) { return n.z() != plane; });
  scan(map, 1.8);
  EXPECT_TRUE(map.ever_free(map.index_of(Vec3(1.05, 0.05, 0.05))));
  EXPECT_FALSE(map.ever_free(map.index_of(Vec3(1.85, 0.05, 0.05))));  // next to unseen space
}

TEST(LocalMap, SomethingWalkingInDoesNotTakeEverFreeFromTheSpaceAheadOfIt) {
  // Returns from something moving that land outside ever-free space are mapped, but the cells
  // next to them stay ever-free, so its next step is recognised as motion (Dynablox).
  LocalMap map = small_map();
  map.set_vertically_unseen([&](const Index3& n) { return n.z() != map.index_of(Vec3(0, 0, 0.05)).z(); });
  for (int i = 0; i < 6; ++i) {
    scan(map, 1.8);
  }
  scan(map, 1.8, -0.05, 0.05, 1.55);
  EXPECT_EQ(map.at(Vec3(1.55, 0.05, 0.05)), Cell::Occupied);
  EXPECT_TRUE(map.ever_free(map.index_of(Vec3(1.45, 0.05, 0.05))));
  map.revoke_ever_free(map.index_of(Vec3(1.45, 0.05, 0.05)));
  EXPECT_FALSE(map.ever_free(map.index_of(Vec3(1.35, 0.05, 0.05))));
}

TEST(LocalMap, TheFaceOfAWallIsNotEverFree) {
  LocalMap map = small_map();
  map.set_vertically_unseen([&](const Index3& n) { return n.z() != map.index_of(Vec3(0, 0, 0.05)).z(); });
  for (int i = 0; i < 8; ++i) {
    scan(map, 1.8, -1.0, 1.0, 1.25);
  }
  EXPECT_EQ(map.at(Vec3(1.25, 0.05, 0.05)), Cell::Occupied);
  EXPECT_FALSE(map.ever_free(map.index_of(Vec3(1.15, 0.05, 0.05))));
  EXPECT_TRUE(map.ever_free(map.index_of(Vec3(0.75, 0.05, 0.05))));
}

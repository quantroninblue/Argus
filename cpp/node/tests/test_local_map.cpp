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

#include <cmath>

#include <gtest/gtest.h>

#include "argus/capabilities/geometry.hpp"

using namespace argus::capabilities;

namespace {

Shape box(Vec3 size, Vec3 xyz = Vec3::Zero(), Vec3 rpy = Vec3::Zero()) {
  Shape s;
  s.type = ShapeType::Box;
  s.size = size;
  s.pose.xyz = xyz;
  s.pose.rpy = rpy;
  return s;
}

}  // namespace

TEST(Geometry, BoxContainsAndIntersects) {
  const Shape s = box(Vec3(2.0, 1.0, 1.0));
  EXPECT_TRUE(s.contains(Vec3(0.9, 0.4, 0.4)));
  EXPECT_FALSE(s.contains(Vec3(1.1, 0.0, 0.0)));
  const auto t = s.intersect(Vec3(-3.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), 0.0, 10.0);
  ASSERT_TRUE(t);
  EXPECT_NEAR(*t, 2.0, 1e-9);
  EXPECT_FALSE(s.intersect(Vec3(-3.0, 2.0, 0.0), Vec3(1.0, 0.0, 0.0), 0.0, 10.0));
  // Segment that stops short of the box.
  EXPECT_FALSE(s.intersect(Vec3(-3.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), 0.0, 1.5));
}

TEST(Geometry, RotatedBox) {
  const Shape s = box(Vec3(2.0, 0.2, 0.2), Vec3::Zero(), Vec3(0.0, 0.0, M_PI / 4));
  EXPECT_TRUE(s.contains(Vec3(0.6, 0.6, 0.0)));
  EXPECT_FALSE(s.contains(Vec3(0.6, -0.6, 0.0)));
}

TEST(Geometry, CylinderSideAndCap) {
  Shape s;
  s.type = ShapeType::Cylinder;
  s.radius = 0.5;
  s.length = 2.0;
  const auto side = s.intersect(Vec3(-2.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), 0.0, 10.0);
  ASSERT_TRUE(side);
  EXPECT_NEAR(*side, 1.5, 1e-9);
  const auto cap = s.intersect(Vec3(0.0, 0.0, 3.0), Vec3(0.0, 0.0, -1.0), 0.0, 10.0);
  ASSERT_TRUE(cap);
  EXPECT_NEAR(*cap, 2.0, 1e-9);
  EXPECT_FALSE(s.intersect(Vec3(-2.0, 0.0, 1.5), Vec3(1.0, 0.0, 0.0), 0.0, 10.0));
}

TEST(Geometry, Sphere) {
  Shape s;
  s.type = ShapeType::Sphere;
  s.radius = 1.0;
  s.pose.xyz = Vec3(5.0, 0.0, 0.0);
  const auto t = s.intersect(Vec3::Zero(), Vec3(1.0, 0.0, 0.0), 0.0, 10.0);
  ASSERT_TRUE(t);
  EXPECT_NEAR(*t, 4.0, 1e-9);
  EXPECT_TRUE(s.contains(Vec3(5.5, 0.0, 0.0)));
}

TEST(Geometry, ConvexHullDropsInteriorPoints) {
  const auto hull = convex_hull({{-1, -1}, {1, -1}, {1, 1}, {-1, 1}, {0, 0}, {0.5, 0.2}, {1, 0}});
  EXPECT_EQ(hull.size(), 4u);
}

TEST(Geometry, RayExitDistance) {
  const std::vector<Vec2> square = convex_hull({{-1, -1}, {1, -1}, {1, 1}, {-1, 1}});
  EXPECT_NEAR(ray_exit_distance(square, Vec2(1, 0)), 1.0, 1e-9);
  EXPECT_NEAR(ray_exit_distance(square, Vec2(1, 1).normalized()), std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(ray_exit_distance(square, Vec2(0, -1)), 1.0, 1e-9);
}

#include <gtest/gtest.h>

#include <cmath>

#include "argus/argo_ros/convert.hpp"

namespace ar = argus::argo_ros;
namespace caps = argus::capabilities;
using argus::node::make_ray_pattern;
using argus::node::RayPatternOptions;

namespace {

caps::SensorSpec spec(caps::SensorKind kind, double h, double v, double min_r, double max_r) {
  caps::SensorSpec s;
  s.id = "s";
  s.kind = kind;
  s.fov_h_deg = h;
  s.fov_v_deg = v;
  s.range_m = caps::Interval{min_r, max_r};
  return s;
}

constexpr double kDeg = M_PI / 180.0;

// Index of the pattern ray closest to a direction given in degrees.
size_t nearest_ray(const argus::node::RayPattern& p, double az_deg, double el_deg) {
  const argus::node::Vec3 want(std::cos(el_deg * kDeg) * std::cos(az_deg * kDeg),
                         std::cos(el_deg * kDeg) * std::sin(az_deg * kDeg), std::sin(el_deg * kDeg));
  size_t best = 0;
  for (size_t i = 0; i < p.size(); ++i) {
    if (p.directions[i].dot(want) > p.directions[best].dot(want)) best = i;
  }
  return best;
}

}  // namespace

TEST(Convert, ScanKeepsTheNearestReturnInEachPatternRay) {
  const auto s = spec(caps::SensorKind::Range2d, 270, 1, 0.1, 12.0);
  const auto pattern = make_ray_pattern(s, RayPatternOptions{});
  ar::Scan scan;
  scan.angle_min = -135 * kDeg;
  scan.angle_increment = 0.25 * kDeg;
  scan.range_min = 0.1;
  scan.range_max = 12.0;
  scan.ranges.assign(1081, 5.0f);
  scan.ranges[(10.0 + 135.0) / 0.25] = 1.0f;          // a thin post at 10 degrees
  scan.ranges[(-60.0 + 135.0) / 0.25] = INFINITY;     // one beam with no return among hits
  for (int i = 0; i < 40; ++i) scan.ranges[(90.0 + 135.0) / 0.25 + i] = INFINITY;  // open area
  for (int i = 0; i < 40; ++i) scan.ranges[(-100.0 + 135.0) / 0.25 + i] = NAN;     // dropout
  const auto out = ar::scan_to_pattern(scan, s, pattern);
  ASSERT_EQ(out.size(), pattern.size());
  EXPECT_FLOAT_EQ(out[nearest_ray(pattern, 10, 0)], 1.0f);
  EXPECT_FLOAT_EQ(out[nearest_ray(pattern, -60, 0)], 5.0f);
  EXPECT_FLOAT_EQ(out[nearest_ray(pattern, 40, 0)], 5.0f);
  EXPECT_TRUE(std::isinf(out[nearest_ray(pattern, 95, 0)]));
  EXPECT_TRUE(std::isnan(out[nearest_ray(pattern, -95, 0)]));
}

TEST(Convert, PointsFillOnlyTheRaysTheyFallIn) {
  const auto s = spec(caps::SensorKind::Range3d, 360, 30, 0.2, 30.0);
  const auto pattern = make_ray_pattern(s, RayPatternOptions{});
  const size_t ahead = nearest_ray(pattern, 0, 0);
  const argus::node::Vec3 d = pattern.directions[ahead];
  std::vector<float> xyz;
  for (double r : {2.0, 3.0, 0.05}) {
    for (int a = 0; a < 3; ++a) xyz.push_back(static_cast<float>(r * d[a]));
  }
  const auto out = ar::points_to_pattern(xyz, s, pattern);
  EXPECT_NEAR(out[ahead], 2.0f, 1e-5);  // nearest of the two, the third is inside min range
  EXPECT_TRUE(std::isnan(out[nearest_ray(pattern, 90, 0)]));
}

TEST(Convert, DepthBecomesRangeAlongEachRay) {
  const auto s = spec(caps::SensorKind::Depth, 87, 58, 0.3, 6.0);
  const auto pattern = make_ray_pattern(s, RayPatternOptions{});
  ar::DepthImage img;
  img.width = 64;
  img.height = 48;
  img.fx = (img.width / 2.0) / std::tan(43.5 * kDeg);
  img.fy = (img.height / 2.0) / std::tan(29.0 * kDeg);
  img.cx = img.width / 2.0;
  img.cy = img.height / 2.0;
  img.depth.assign(img.width * img.height, 3.0f);
  for (int u = 0; u < 10; ++u) {
    for (int v = 0; v < img.height; ++v) img.depth[v * img.width + u] = 0.0f;  // invalid left edge
  }
  const auto out = ar::depth_to_pattern(img, s, pattern);
  const size_t centre = nearest_ray(pattern, 0, 0);
  EXPECT_NEAR(out[centre] * pattern.directions[centre].x(), 3.0, 1e-3);
  const size_t off = nearest_ray(pattern, -30, -20);
  EXPECT_NEAR(out[off] * pattern.directions[off].x(), 3.0, 1e-3);
  EXPECT_TRUE(std::isnan(out[nearest_ray(pattern, 42, 0)]));
}

TEST(Convert, YawFromQuaternion) {
  EXPECT_NEAR(ar::yaw_of(0, 0, std::sin(0.5), std::cos(0.5)), 1.0, 1e-12);
  EXPECT_NEAR(ar::yaw_of(0, 0, std::sin(-1.4), std::cos(-1.4)), -2.8, 1e-12);
}

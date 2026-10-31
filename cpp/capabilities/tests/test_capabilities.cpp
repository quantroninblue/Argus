#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "argus/capabilities/capabilities.hpp"

using namespace argus::capabilities;

namespace {

Capabilities example(const std::string& name) {
  return derive_capabilities(load_profile_file(std::string(ARGUS_EXAMPLES_DIR) + "/" + name));
}

const CoverageBin& bin_at(const SensingCapability& s, double azimuth, double elevation = 0.0) {
  for (const auto& b : s.bins) {
    if (std::abs(b.azimuth_deg - azimuth) < 1e-6 && std::abs(b.elevation_deg - elevation) < 1e-6) {
      return b;
    }
  }
  throw std::runtime_error("no such bin");
}

// A box body with one 360 degree scanner in front of it, level with the box.
const char* kSelfOccluded = R"(
argus_profile: 1
id: occluded
geometry:
  ground_z_m: 0.0
  shapes:
    - {type: box, size_m: [0.6, 0.6, 0.4], pose: {xyz_m: [0.0, 0.0, 0.25]}}
  frames:
    scanner: {xyz_m: [0.35, 0.0, 0.25]}
motion:
  space: planar
  can_reverse: false
  can_strafe: false
  collision_padding_m: 0.0
  limits:
    x: {velocity_mps: [0.0, 1.0], accel_mps2: 1.0, decel_mps2: 1.0}
    y: {velocity_mps: [0.0, 0.0]}
    z: {velocity_mps: [0.0, 0.0]}
    yaw: {rate_radps: 1.0, accel_radps2: 1.0}
control: {interface: velocity, transport: ros2, endpoint: /cmd_vel, rate_hz: 20, latency_s: 0.1, controller_timeout_s: 0.5}
failsafe: {link_lost_after_s: 2.0, on_link_lost: stop, uncovered_motion: forbid}
sensors:
  - id: scanner
    kind: range_2d
    frame: scanner
    source: {transport: ros2, endpoint: /scan}
    rate_hz: 10
    range_m: [0.05, 8.0]
    fov_deg: {h: 360, v: 1.0}
compute: {board: test, accelerator: none}
provenance:
  motion.limits.x.decel_mps2: llm_draft
)";

}  // namespace

TEST(Capabilities, DifferentialMotion) {
  const Capabilities c = example("planar_differential.yaml");
  EXPECT_EQ(c.motion.space, MotionSpace::Planar);
  EXPECT_TRUE(c.motion.can_turn_in_place);
  EXPECT_FALSE(c.motion.can_strafe);
  EXPECT_FALSE(c.motion.can_hover);
  EXPECT_FALSE(c.motion.y.enabled());
  // 1.0 m/s with 0.08 s latency and 1.2 m/s^2 braking.
  EXPECT_NEAR(c.motion.stopping_distance_m(Axis::X, 1.0), 0.08 + 1.0 / 2.4, 1e-12);
  EXPECT_NEAR(c.motion.stopping_distance_m(Axis::X, -0.3), 0.3 * 0.08 + 0.09 / 2.4, 1e-12);
  EXPECT_EQ(c.motion.stopping_distance_m(Axis::Y, 0.0), 0.0);
  EXPECT_EQ(c.motion.stopping_distance_m(Axis::Y, 0.1), kNever);
  EXPECT_TRUE(c.safety_ready);
}

TEST(Capabilities, FootprintIncludesPadding) {
  const Capabilities c = example("planar_differential.yaml");
  EXPECT_NEAR(c.collision.edge_distance_m(0.0), 0.35, 0.002);
  EXPECT_NEAR(c.collision.edge_distance_m(M_PI / 2), 0.275, 0.002);
  EXPECT_NEAR(c.collision.aabb_max.z(), 0.70, 1e-9);
}

TEST(Capabilities, DifferentialCoverageHasRearBlindSpot) {
  const Capabilities c = example("planar_differential.yaml");
  ASSERT_EQ(c.sensing.bins.size(), 72u);
  EXPECT_TRUE(bin_at(c.sensing, 0.0).seen());
  EXPECT_TRUE(bin_at(c.sensing, 90.0).seen());
  EXPECT_FALSE(bin_at(c.sensing, 180.0).seen());
  // Ahead, the downward camera fills in low obstacles the lidar plane misses.
  EXPECT_LT(bin_at(c.sensing, 0.0).full_height_from_m, kNever);
  // Every band ahead stays in view out to somewhere inside the camera's 6 m range.
  EXPECT_GT(bin_at(c.sensing, 0.0).full_height_reach_m, 1.0);
  EXPECT_LE(bin_at(c.sensing, 0.0).full_height_reach_m, 6.0);
  // To the side only the lidar plane looks, so low obstacles stay unseen.
  const CoverageBin& side = bin_at(c.sensing, 90.0);
  EXPECT_EQ(side.full_height_from_m, kNever);
  EXPECT_EQ(side.full_height_reach_m, 0.0);
  ASSERT_FALSE(side.unseen_bands.empty());
  EXPECT_NEAR(side.unseen_bands.front().low_m, 0.05, 1e-9);
  EXPECT_EQ(side.sensors, std::vector<std::string>{"front_lidar"});
}

TEST(Capabilities, OmnidirectionalSeesAllAround) {
  const Capabilities c = example("planar_omnidirectional.yaml");
  EXPECT_TRUE(c.motion.can_strafe);
  EXPECT_DOUBLE_EQ(c.sensing.seen_fraction(), 1.0);
}

TEST(Capabilities, AerialCoverageByElevation) {
  const Capabilities c = example("aerial_multirotor.yaml");
  EXPECT_TRUE(c.motion.can_hover);
  EXPECT_EQ(c.sensing.bins.size(), 13u * 72u);
  EXPECT_TRUE(bin_at(c.sensing, 0.0, 0.0).seen());
  EXPECT_TRUE(bin_at(c.sensing, 0.0, -90.0).seen());
  EXPECT_FALSE(bin_at(c.sensing, 0.0, 90.0).seen());
  EXPECT_GT(c.collision.bounding_radius_m, 0.25 + 0.12 + 0.10);
  EXPECT_DOUBLE_EQ(c.collision.edge_distance_m(1.0, 0.5), c.collision.bounding_radius_m);
}

TEST(Capabilities, BodyBlocksItsOwnSensor) {
  const Capabilities c = derive_capabilities(load_profile_text(kSelfOccluded));
  EXPECT_TRUE(bin_at(c.sensing, 0.0).seen());
  EXPECT_FALSE(bin_at(c.sensing, 180.0).seen());
  EXPECT_FALSE(c.safety_ready);
  EXPECT_EQ(c.unreviewed_fields, std::vector<std::string>{"motion.limits.x.decel_mps2"});
}

TEST(Capabilities, LocalizationAndCompute) {
  const Capabilities planar = example("planar_differential.yaml");
  EXPECT_EQ(planar.localization.sources(),
            (std::vector<std::string>{"inertial", "wheel_odometry", "visual", "range"}));
  EXPECT_EQ(planar.compute.inference_backends, (std::vector<std::string>{"rknn", "onnx_cpu"}));
  const Capabilities aerial = example("aerial_multirotor.yaml");
  EXPECT_TRUE(aerial.localization.external_odometry);
  EXPECT_FALSE(aerial.localization.wheel_odometry);
}

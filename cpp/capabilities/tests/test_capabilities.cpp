#include <cmath>
#include <fstream>
#include <sstream>
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

TEST(Capabilities, OneScanPlaneSeesTheWholeBodyHeightOnlyUnderADeclaredObstacleHeight) {
  // kSelfOccluded: a 360 degree scan plane at 0.25 m on a body from 0.05 to 0.45 m.
  const Capabilities plain = derive_capabilities(load_profile_text(kSelfOccluded));
  const CoverageBin& ahead = bin_at(plain.sensing, 0.0);
  EXPECT_TRUE(ahead.seen());                          // the plane does see something
  EXPECT_EQ(ahead.full_height_from_m, kNever);         // but not obstacles below or above it
  EXPECT_FALSE(ahead.unseen_bands.empty());

  auto with_assumption = [](double h) {
    return derive_capabilities(load_profile_text(std::string(kSelfOccluded) +
                                                 "assumptions: {min_obstacle_height_m: " + std::to_string(h) + "}\n"));
  };
  // Obstacles at least 0.30 m tall: the plane at 0.25 m sees each one, but the body's upper
  // part (0.30 to 0.45 m) still meets nothing it can see.
  EXPECT_EQ(bin_at(with_assumption(0.30).sensing, 0.0).full_height_from_m, kNever);
  // As tall as the body or taller: one plane is enough.
  const Capabilities tall = with_assumption(0.50);
  EXPECT_NE(bin_at(tall.sensing, 0.0).full_height_from_m, kNever);
  EXPECT_DOUBLE_EQ(tall.assumed_obstacle_height_m, 0.50);
  EXPECT_DOUBLE_EQ(tall.min_obstacle_height_m, 0.05);
}

namespace {

std::string example_text(const std::string& name) {
  std::ifstream in(std::string(ARGUS_EXAMPLES_DIR) + "/" + name);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string replace_once(std::string text, const std::string& from, const std::string& to) {
  const size_t at = text.find(from);
  EXPECT_NE(at, std::string::npos) << from;
  return at == std::string::npos ? text : text.replace(at, from.size(), to);
}

}  // namespace

TEST(Capabilities, ADroneThatCannotSeeUpMustDeclareHowHighItMayFly) {
  const std::string text = example_text("aerial_multirotor.yaml");
  const Capabilities declared = derive_capabilities(load_profile_text(text));
  EXPECT_FALSE(declared.sees_above);
  EXPECT_DOUBLE_EQ(declared.max_flight_height_m, 2.5);
  EXPECT_TRUE(declared.safety_ready);

  const Capabilities missing =
      derive_capabilities(load_profile_text(replace_once(text, "  max_flight_height_m: 2.5\n", "")));
  EXPECT_FALSE(missing.safety_ready);
  ASSERT_EQ(missing.missing_fields.size(), 1u);
  EXPECT_NE(missing.missing_fields[0].find("max_flight_height_m"), std::string::npos);
}

TEST(Capabilities, ADroneWithAnUpwardSensorSeesAboveAndNeedsNoDeclaredHeight) {
  // Turn the downward depth camera to face up.
  std::string text = example_text("aerial_multirotor.yaml");
  text = replace_once(text, "rpy_rad: [0.0, 1.5708, 0.0]", "rpy_rad: [0.0, -1.5708, 0.0]");
  text = replace_once(text, "  max_flight_height_m: 2.5\n", "");
  const Capabilities c = derive_capabilities(load_profile_text(text));
  EXPECT_TRUE(c.sees_above);
  EXPECT_TRUE(c.safety_ready);
  EXPECT_TRUE(c.missing_fields.empty());
}

TEST(Capabilities, SparseRaysStopCoverageWhereAThinObstacleFitsBetweenThem) {
  // The front lidar and depth camera with their rays 1 degree apart: a 5 cm pole fits
  // between two of them beyond 0.05 / (2 tan 0.5 deg), about 2.9 m. Declaring 10 cm as the
  // thinnest obstacle doubles that.
  std::string text = example_text("planar_differential.yaml");
  text = replace_once(text, "    fov_deg: {h: 270, v: 1.0}\n",
                      "    fov_deg: {h: 270, v: 1.0}\n    angular_resolution_deg: 1.0\n");
  text = replace_once(text, "    fov_deg: {h: 87, v: 58}\n",
                      "    fov_deg: {h: 87, v: 58}\n    angular_resolution_deg: 1.0\n");
  const auto reach_ahead = [](const Capabilities& c) {
    for (const auto& b : c.sensing.bins) {
      if (b.azimuth_deg == 0.0) return b.max_seen_m + b.edge_distance_m;
    }
    return 0.0;
  };
  const Capabilities fine = derive_capabilities(load_profile_text(example_text("planar_differential.yaml")));
  const Capabilities sparse = derive_capabilities(load_profile_text(text));
  const double thin = 0.05 / (2.0 * std::tan(0.5 * M_PI / 180.0));
  EXPECT_GT(reach_ahead(fine), thin + 1.0);
  EXPECT_LE(reach_ahead(sparse), thin + 0.4);  // ranges count from the sensor, a little ahead of the base
  const Capabilities wider = derive_capabilities(load_profile_text(
      replace_once(text, "assumptions:\n", "assumptions:\n  min_obstacle_width_m: 0.1\n")));
  EXPECT_NEAR(reach_ahead(wider), 2.0 * thin, 0.3);
}

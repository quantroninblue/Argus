#include <fstream>
#include <functional>
#include <sstream>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "argus/capabilities/profile.hpp"

using namespace argus::capabilities;

namespace {

std::string read_example(const std::string& name) {
  std::ifstream in(std::string(ARGUS_EXAMPLES_DIR) + "/" + name);
  std::stringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

// Loads the differential example, applies an edit and returns the problems found.
std::vector<std::string> problems_after(const std::function<void(YAML::Node&)>& edit) {
  YAML::Node root = YAML::Load(read_example("planar_differential.yaml"));
  edit(root);
  return validate_profile_text(YAML::Dump(root));
}

bool mentions(const std::vector<std::string>& problems, const std::string& text) {
  for (const auto& p : problems) {
    if (p.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(Profile, ExamplesLoad) {
  for (const char* name :
       {"planar_differential.yaml", "planar_omnidirectional.yaml", "aerial_multirotor.yaml"}) {
    EXPECT_TRUE(validate_profile_text(read_example(name)).empty()) << name;
  }
  const Profile p = load_profile_file(std::string(ARGUS_EXAMPLES_DIR) + "/planar_differential.yaml");
  EXPECT_EQ(p.id, "example-planar-differential");
  EXPECT_EQ(p.motion.space, MotionSpace::Planar);
  EXPECT_DOUBLE_EQ(p.motion.x.velocity_mps.max, 1.0);
  EXPECT_EQ(p.sensors.size(), 4u);
  ASSERT_NE(p.sensor("front_lidar"), nullptr);
  EXPECT_DOUBLE_EQ(*p.sensor("front_lidar")->fov_h_deg, 270.0);
  EXPECT_NEAR(p.sensor("imu")->max_age_s, 0.03, 1e-12);
  EXPECT_EQ(p.provenance.at("control.latency_s"), Provenance::Declared);
}

TEST(Profile, ReportsEveryProblemAtOnce) {
  const auto problems = problems_after([](YAML::Node& root) {
    root.remove("motion");
    root["id"] = "Not Valid";
    root["colour"] = "red";
  });
  EXPECT_TRUE(mentions(problems, "motion: required"));
  EXPECT_TRUE(mentions(problems, "id: ids use"));
  EXPECT_TRUE(mentions(problems, "colour: unknown field"));
}

TEST(Profile, RejectsInconsistentMotion) {
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) {
                         r["motion"]["limits"]["y"]["velocity_mps"][1] = 0.5;
                         r["motion"]["limits"]["y"]["accel_mps2"] = 1.0;
                         r["motion"]["limits"]["y"]["decel_mps2"] = 1.0;
                       }),
                       "can_strafe is false"));
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) { r["motion"]["can_reverse"] = false; }),
                       "can_reverse is false"));
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) {
                         r["motion"]["limits"]["z"]["velocity_mps"][1] = 0.5;
                         r["motion"]["limits"]["z"]["accel_mps2"] = 1.0;
                         r["motion"]["limits"]["z"]["decel_mps2"] = 1.0;
                       }),
                       "cannot move vertically"));
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) {
                         r["motion"]["limits"]["x"]["velocity_mps"][0] = 0.2;
                       }),
                       "must include zero"));
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) {
                         r["motion"]["limits"]["x"].remove("decel_mps2");
                       }),
                       "decel_mps2: required"));
}

TEST(Profile, RejectsBrokenSensorsAndFailsafe) {
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) { r["sensors"][0]["frame"] = "nowhere"; }),
                       "not defined in geometry.frames"));
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) { r["sensors"][1]["id"] = "front_lidar"; }),
                       "duplicate sensor id"));
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) { r["sensors"][0].remove("fov_deg"); }),
                       "fov_deg: required"));
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) { r["failsafe"]["on_link_lost"] = "land"; }),
                       "only valid for aerial"));
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) {
                         r["failsafe"]["uncovered_motion"] = "creep";
                       }),
                       "creep_speed_mps: required"));
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) {
                         r["failsafe"]["uncovered_motion"] = "creep";
                         r["failsafe"]["creep_speed_mps"] = 5.0;
                       }),
                       "faster than the body's top speed"));
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) { r["geometry"].remove("ground_z_m"); }),
                       "required for planar bodies"));
}

TEST(Profile, ProvenanceMustNameRealFields) {
  EXPECT_TRUE(problems_after([](YAML::Node& r) {
                r["provenance"]["sensors.front_lidar.range_m"] = "calibrated";
              }).empty());
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) {
                         r["provenance"]["motion.limits.x.top_speed"] = "declared";
                       }),
                       "does not name a field"));
  EXPECT_TRUE(mentions(problems_after([](YAML::Node& r) {
                         r["provenance"]["control.latency_s"] = "guessed";
                       }),
                       "must be one of"));
}

TEST(Profile, RejectsBadYaml) {
  const auto problems = validate_profile_text("id: [unclosed");
  ASSERT_FALSE(problems.empty());
  EXPECT_TRUE(mentions(problems, "not valid YAML"));
}

#pragma once

#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "argus/capabilities/geometry.hpp"

namespace argus::capabilities {

inline constexpr int kProfileFormatVersion = 1;

enum class MotionSpace { Planar, Aerial };
enum class ControlInterface { Velocity, PoseSetpoint, Waypoint };
enum class Transport { Ros2, Mavlink, Serial, Custom };
enum class LinkLostAction { Stop, Hold, FinishLegThenStop, ReturnHome, Land };
enum class UncoveredMotion { Forbid, Creep };
enum class SensorKind { Range2d, Range3d, Depth, Rgb, Imu, WheelOdom, Odometry, Contact };
enum class Accelerator { None, Rk3588Npu, Hailo, Cuda };
enum class Provenance { Declared, Cad, Calibrated, LlmDraft, LlmDraftReviewed };

struct Interval {
  double min = 0.0;
  double max = 0.0;
};

struct AxisLimits {
  Interval velocity_mps;
  double accel_mps2 = 0.0;
  double decel_mps2 = 0.0;

  bool moves() const { return velocity_mps.min != 0.0 || velocity_mps.max != 0.0; }
};

struct MotionSpec {
  MotionSpace space = MotionSpace::Planar;
  bool can_reverse = false;
  bool can_strafe = false;
  double min_turn_radius_m = 0.0;
  AxisLimits x;
  AxisLimits y;
  AxisLimits z;
  double yaw_rate_radps = 0.0;
  double yaw_accel_radps2 = 0.0;
  double collision_padding_m = 0.0;
};

struct GeometrySpec {
  std::optional<double> ground_z_m;
  std::vector<Shape> shapes;
  std::map<std::string, Pose> frames;
};

struct ControlSpec {
  ControlInterface interface = ControlInterface::Velocity;
  Transport transport = Transport::Ros2;
  std::string endpoint;
  double rate_hz = 0.0;
  double latency_s = 0.0;
  double controller_timeout_s = 0.0;
};

struct FailsafeSpec {
  double link_lost_after_s = 0.0;
  LinkLostAction on_link_lost = LinkLostAction::Stop;
  UncoveredMotion uncovered_motion = UncoveredMotion::Forbid;
  double creep_speed_mps = 0.0;
};

struct SensorSpec {
  std::string id;
  SensorKind kind = SensorKind::Imu;
  std::string frame;
  Transport transport = Transport::Ros2;
  std::string endpoint;
  double rate_hz = 0.0;
  double max_age_s = 0.0;
  std::optional<Interval> range_m;
  std::optional<double> fov_h_deg;
  std::optional<double> fov_v_deg;
};

struct ComputeSpec {
  std::string board;
  Accelerator accelerator = Accelerator::None;
};

struct Profile {
  int format_version = kProfileFormatVersion;
  std::string id;
  std::string name;
  std::string description_file;
  GeometrySpec geometry;
  MotionSpec motion;
  ControlSpec control;
  FailsafeSpec failsafe;
  std::vector<SensorSpec> sensors;
  ComputeSpec compute;
  // Declared: obstacles where the robot works stand on the ground and are at least this tall, so
  // seeing any height up to it means seeing the whole column below. Lets a robot whose only
  // sensor is one scan plane drive normally; unset, nothing is assumed.
  std::optional<double> assumed_min_obstacle_height_m;
  std::map<std::string, Provenance> provenance;

  const SensorSpec* sensor(const std::string& id) const;
};

// Every problem found in a profile, not just the first one.
class ProfileError : public std::runtime_error {
 public:
  explicit ProfileError(std::vector<std::string> problems);
  const std::vector<std::string>& problems() const { return problems_; }

 private:
  std::vector<std::string> problems_;
};

Profile load_profile_text(const std::string& yaml_text);
Profile load_profile_file(const std::string& path);

// Empty when the profile is valid.
std::vector<std::string> validate_profile_text(const std::string& yaml_text);

bool is_range_sensor(SensorKind kind);

std::string to_string(MotionSpace value);
std::string to_string(ControlInterface value);
std::string to_string(Transport value);
std::string to_string(LinkLostAction value);
std::string to_string(UncoveredMotion value);
std::string to_string(SensorKind value);
std::string to_string(Accelerator value);
std::string to_string(Provenance value);

}  // namespace argus::capabilities

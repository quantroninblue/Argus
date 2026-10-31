#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "argus/capabilities/geometry.hpp"

namespace argus::node {

using capabilities::Vec2;
using capabilities::Vec3;

// Pose of the base in the odom frame. Bodies are treated as level: planar bodies drive on a
// floor and multirotors keep their own attitude, so position and yaw are enough to navigate.
struct Pose4 {
  Vec3 position = Vec3::Zero();
  double yaw = 0.0;

  Eigen::Isometry3d transform() const;
  Vec3 to_odom(const Vec3& base_point) const;
  Vec3 rotate(const Vec3& base_vector) const;
};

// Velocity of the base, expressed in the base frame.
struct Twist {
  Vec3 linear = Vec3::Zero();
  double yaw_rate = 0.0;

  bool is_zero() const { return linear.isZero() && yaw_rate == 0.0; }
};

struct Odometry {
  double stamp = 0.0;
  Pose4 pose;
  Twist twist;
  bool healthy = true;
};

// One frame from a range sensor: a rows x cols grid of ranges in metres, row-major, in the
// order of the sensor's RayPattern. inf means no return within max range, NaN means invalid.
struct RangeFrame {
  double stamp = 0.0;
  std::string sensor_id;
  std::vector<float> ranges;
};

struct Waypoint {
  std::string id;
  Vec3 position = Vec3::Zero();  // odom frame
  double tolerance_m = 0.25;
  double speed_cap_mps = 0.0;  // 0 means no cap beyond the body's limits
};

struct Plan {
  uint64_t generation = 0;
  double expires = 0.0;
  std::vector<Waypoint> waypoints;
};

// Same causes and meaning as FailureCause in the protocol.
enum class FailureCause {
  None,
  DeadEndObserved,
  SearchedNoGoal,
  TemporaryObstruction,
  ControllerNoProgress,
  LocalizationInvalid,
  SensorInvalid,
  ComputeDeadlineMiss,
  LinkLost,
};

enum class EventKind { PlanAccepted, PlanRejected, WaypointReached, PlanComplete, PlanExpired, Blocked };

struct Event {
  double stamp = 0.0;
  EventKind kind = EventKind::Blocked;
  std::string waypoint_id;
  FailureCause cause = FailureCause::None;
  std::string detail;
};

// What goes to the robot controller. Which fields matter depends on the control interface.
struct Command {
  double stamp = 0.0;
  bool stop = true;
  // Velocity interface: base-frame velocity.
  Twist velocity;
  // Pose setpoint interface: target pose in odom with velocity feed-forward (odom frame).
  Pose4 setpoint;
  Vec3 setpoint_velocity = Vec3::Zero();
  double setpoint_yaw_rate = 0.0;
};

std::string to_string(FailureCause cause);
std::string to_string(EventKind kind);

double wrap_angle(double a);

}  // namespace argus::node

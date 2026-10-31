#pragma once

#include <limits>
#include <string>
#include <vector>

#include "argus/capabilities/geometry.hpp"
#include "argus/capabilities/profile.hpp"

namespace argus::capabilities {

inline constexpr double kNever = std::numeric_limits<double>::infinity();

enum class Axis { X, Y, Z };

struct AxisCapability {
  double min_mps = 0.0;
  double max_mps = 0.0;
  double accel_mps2 = 0.0;
  double decel_mps2 = 0.0;

  bool enabled() const { return min_mps != 0.0 || max_mps != 0.0; }
};

struct MotionCapability {
  MotionSpace space = MotionSpace::Planar;
  bool can_reverse = false;
  bool can_strafe = false;
  bool can_turn_in_place = false;
  bool can_hover = false;
  double min_turn_radius_m = 0.0;
  AxisCapability x;
  AxisCapability y;
  AxisCapability z;
  double yaw_rate_max_radps = 0.0;
  double yaw_accel_radps2 = 0.0;
  double latency_s = 0.0;

  const AxisCapability& axis(Axis a) const;

  // Distance covered from the moment a stop is decided until the body is at rest:
  // travel during command latency plus braking at the axis deceleration.
  double stopping_distance_m(Axis a, double speed_mps) const;
};

struct CollisionCapability {
  Vec3 aabb_min = Vec3::Zero();
  Vec3 aabb_max = Vec3::Zero();
  // Planar bodies: convex footprint in the base frame including padding.
  std::vector<Vec2> footprint;
  // Radius around the base origin that contains the whole body including padding.
  double bounding_radius_m = 0.0;
  // The same two without padding: the body itself.
  std::vector<Vec2> body_footprint;
  double body_radius_m = 0.0;
  double padding_m = 0.0;
  // Distance from the base origin to the nearest padded footprint edge (planar bodies), or the
  // padded bounding radius (aerial bodies).
  double inscribed_radius_m = 0.0;
  MotionSpace space = MotionSpace::Planar;

  // Distance from the base origin to the padded body edge along a bearing.
  double edge_distance_m(double azimuth_rad, double elevation_rad = 0.0) const;
};

// Height range above the ground, used for planar bodies.
struct HeightBand {
  double low_m = 0.0;
  double high_m = 0.0;
};

// What the range sensors can see along one bearing from the base origin.
// Distances are measured outward from the padded body edge.
struct CoverageBin {
  double azimuth_deg = 0.0;
  double elevation_deg = 0.0;
  double edge_distance_m = 0.0;
  double first_seen_m = kNever;
  double max_seen_m = 0.0;
  // Planar only: distance by which every height band has been seen at least once.
  double full_height_from_m = kNever;
  // Farthest distance at which every height band is still seen (for aerial bodies, the
  // farthest distance seen). An obstacle in this direction is noticed by this distance at the
  // latest, so motion this way must be able to stop within it. 0 when some band is never seen,
  // or when the sensors only start seeing every band farther than `max_near_gap_m` from the
  // edge (the body's own parts blocking the view, say): space that close is never seen from
  // where the body is, so this is not a direction it can move in.
  double full_height_reach_m = 0.0;
  // Planar only: bands no sensor sees anywhere along this bearing.
  std::vector<HeightBand> unseen_bands;
  std::vector<std::string> sensors;

  bool seen() const { return first_seen_m != kNever; }
};

struct SensingCapability {
  double evaluation_range_m = 0.0;
  double sample_step_m = 0.0;
  std::vector<HeightBand> bands;
  std::vector<CoverageBin> bins;

  double seen_fraction() const;
  double full_height_fraction() const;
};

struct LocalizationCapability {
  bool inertial = false;
  bool wheel_odometry = false;
  bool external_odometry = false;
  bool visual = false;
  bool range = false;

  std::vector<std::string> sources() const;
};

struct ComputeCapability {
  std::string board;
  Accelerator accelerator = Accelerator::None;
  std::vector<std::string> inference_backends;
};

struct Capabilities {
  std::string profile_id;
  MotionCapability motion;
  CollisionCapability collision;
  SensingCapability sensing;
  LocalizationCapability localization;
  ComputeCapability compute;
  ControlSpec control;
  FailsafeSpec failsafe;
  // Obstacles shorter than this are treated as ground.
  double min_obstacle_height_m = 0.05;
  // The profile's declared minimum obstacle height (profile.hpp), 0 when none.
  double assumed_obstacle_height_m = 0.0;
  // Declared: space this far from the body where Argo starts is clear.
  double assumed_clear_at_start_m = 0.0;
  // False while any field is still an unreviewed LLM draft.
  bool safety_ready = true;
  std::vector<std::string> unreviewed_fields;
};

struct DeriveOptions {
  double azimuth_step_deg = 5.0;
  double elevation_step_deg = 15.0;
  double evaluation_range_m = 10.0;
  double sample_step_m = 0.05;
  double band_height_m = 0.1;
  double min_obstacle_height_m = 0.05;
  // Largest blind gap next to the body's edge a covered direction may have. Gaps up to this
  // (a depth camera not seeing the ground at the bumper, a minimum range) were seen a moment
  // earlier from farther back, and Argo keeps its views that far out to check them.
  double max_near_gap_m = 1.5;
};

Capabilities derive_capabilities(const Profile& profile, const DeriveOptions& options = {});

}  // namespace argus::capabilities

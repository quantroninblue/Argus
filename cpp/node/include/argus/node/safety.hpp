#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "argus/capabilities/capabilities.hpp"
#include "argus/node/body.hpp"
#include "argus/node/local_map.hpp"
#include "argus/node/motion.hpp"
#include "argus/node/tracker.hpp"

namespace argus::node {

// Why the supervisor changed a command. Several can apply at once.
enum SafetyFlag : uint32_t {
  kNoTarget = 1u << 0,
  kDeadline = 1u << 1,
  kLocalization = 1u << 2,
  kNotSafetyReady = 1u << 3,
  kClamped = 1u << 4,
  kUncovered = 1u << 5,
  kStaleSensor = 1u << 6,
  kCreep = 1u << 7,
  kSlowed = 1u << 8,
  kBlocked = 1u << 9,
  kShortSight = 1u << 10,
  // Argo is holding still: something that was moving has stopped on its way (node.hpp).
  kYielding = 1u << 11,
  // Speed capped because something that moves is close (SafetyConfig::people_*).
  kNearPeople = 1u << 12,
  // Climb capped by the declared highest flight height.
  kCeiling = 1u << 13,
  // Odometry disagrees with what the range sensors show of the robot's own motion
  // (Node: odometry cross-check); speed capped until they agree again.
  kOdometrySuspect = 1u << 14,
};

std::vector<std::string> flag_names(uint32_t flags);

struct SafetyConfig {
  // Commands built from inputs older than this are replaced by a stop. 0: derived from the
  // controller timeout and planner period.
  double command_lease_s = 0.0;
  double odometry_max_age_s = 0.25;
  // Share of the swept body that may be unknown and never looked at by a sensor before the
  // envelope counts as unknown. Cells between a frame's rays do not count (view.hpp).
  double max_unknown_fraction = 0.0;
  double envelope_spacing_m = 0.05;
  // Longest stretch a body may creep through directions it cannot see before it has to move
  // where it can again. Capped further at the body's own length.
  double creep_distance_m = 0.5;
  // Clearance kept from a moving obstacle's predicted position, beyond both radii.
  double mover_margin_m = 0.15;
  // Widening of a moving thing's predicted spot by its tracker uncertainty, as PANTHER does
  // (MIT ACL, `norminv_prob`): this many standard deviations of position spread at the time
  // looked ahead to, at most mover_widen_max_m. Covers turns and speed changes the constant
  // velocity prediction misses.
  double mover_sigmas = 1.0;
  double mover_widen_max_m = 0.3;
  // Slower near things that move (people, carts), as Nav2's collision monitor slowdown zones
  // and ISO 3691-4's reduced speed near people: within people_near_m of one (gap between
  // body and it) at most people_speed_mps, rising linearly to the body's own limit at
  // people_far_m. Only for tracks that have moved; others are plain obstacles.
  double people_near_m = 0.5;
  double people_far_m = 2.0;
  double people_speed_mps = 0.3;
  // Aerial bodies: the base never flies above this height in odom (set by Node from the
  // profile's assumptions.max_flight_height_m); climbs are capped so they stop below it.
  double ceiling_z_m = std::numeric_limits<double>::quiet_NaN();
  // Top speed while odometry is suspect: the map and every stopping distance rest on it.
  double odometry_suspect_speed_mps = 0.25;
};

// Time of the latest frame from each range sensor, and how old a frame may get.
struct SensorClock {
  std::map<std::string, double> last_frame;
  std::map<std::string, double> max_age;

  bool fresh(const std::string& id, double now) const;
};

struct SafetyInputs {
  const Odometry& odom;
  const LocalMap& map;
  const SensorClock& sensors;
  double now;
  // Things moving near the robot, predicted to `now` (tracker.hpp). Each is taken to keep its
  // velocity: people are not assumed to brake for the robot.
  const std::vector<TrackedObstacle>* movers = nullptr;
  bool odometry_suspect = false;
};

enum class Coverage { Covered, Stale, Uncovered };

// Last check before a command reaches the robot controller. It can only slow or stop motion.
class SafetySupervisor {
 public:
  SafetySupervisor(const capabilities::Capabilities& caps, const BodyVolume& body,
                   const MotionModel& motion, const SafetyConfig& config, double control_period_s,
                   double planner_period_s, const std::map<std::string, double>& sensor_max_age);

  struct Verdict {
    Twist velocity;
    uint32_t flags = 0;
  };

  // Checks a proposed velocity computed from inputs stamped `inputs_stamp`.
  Verdict check(const Twist& proposed, double inputs_stamp, const SafetyInputs& in);

  // What the coverage and envelope checks would make of `v` right now, without acceleration
  // limits. The planner uses this to pick a candidate the supervisor will not override.
  Twist preview(const Twist& v, const SafetyInputs& in, uint32_t& flags) const;

  // Whether `v` would pass those checks unchanged.
  bool admits(const Twist& v, const SafetyInputs& in) const;

  // Coverage of a direction of travel in the base frame.
  Coverage coverage(const Vec3& direction, const SafetyInputs& in) const;

  // Fastest speed along a covered direction that still stops within the distance at which the
  // sensors see every height band, allowing for frame age, the control period and latency.
  double sight_speed_limit(const Vec3& direction) const;

  // Whether the profile lets the body creep where it cannot see, and how much of the current
  // creep stretch is left. A body with no range sensor never creeps.
  bool creep_allowed() const { return creep_allowed_; }
  bool creep_available() const { return creep_allowed_ && creep_left_ > 0.0; }
  double creep_left_m() const { return creep_left_; }
  double creep_budget_m() const { return creep_budget_; }

  double lease_s() const { return lease_; }
  const Twist& last() const { return last_; }

 private:
  bool envelope_clear(const Twist& v, const SafetyInputs& in, bool occupied_only, bool* blind_above = nullptr) const;
  bool movers_clear(const std::vector<Pose4>& path, const std::vector<double>& times,
                    const SafetyInputs& in) const;
  // Highest speed allowed this close to things that have moved; infinity when none is near.
  double people_speed_limit(const SafetyInputs& in) const;
  // Whether a body at height z can reach the obstacle (always for planar bodies).
  bool reaches_height(const TrackedObstacle& m, double z, double margin) const;
  Twist limit_acceleration(const Twist& v) const;
  void track_creep(const Vec3& position);
  Twist gate(const Twist& v, const SafetyInputs& in, uint32_t& flags) const;

  const capabilities::Capabilities& caps_;
  const BodyVolume& body_;
  const MotionModel& motion_;
  SafetyConfig config_;
  double period_ = 0.02;
  double lease_ = 0.3;
  std::map<std::string, double> sensor_max_age_;
  // Per coverage bin: how far ahead the whole strip the body sweeps moving that way is seen,
  // measured from its edge. The bin's own full-height reach, shortened where a neighbouring
  // bearing inside the strip sees less far (a wide body passes those bearings too).
  std::vector<double> corridor_reach_;
  void derive_corridor_reach();
  Twist last_;
  bool creep_allowed_ = false;
  double creep_budget_ = 0.0;
  double creep_left_ = 0.0;
  bool last_creep_ = false;
  bool have_last_position_ = false;
  Vec3 last_position_ = Vec3::Zero();
};

}  // namespace argus::node

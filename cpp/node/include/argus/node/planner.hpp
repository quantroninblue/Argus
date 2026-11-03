#pragma once

#include <limits>
#include <vector>

#include "argus/capabilities/capabilities.hpp"
#include "argus/node/body.hpp"
#include "argus/node/hybrid_search.hpp"
#include "argus/node/motion.hpp"
#include "argus/node/safety.hpp"
#include "argus/node/workspace.hpp"

namespace argus::node {

struct PlannerConfig {
  // Argo's own path keeps within this distance of the hub's legs (from where the leg began,
  // to the waypoint, to the next one): it goes round what is in the way and rejoins the hub's
  // route, and never finds a route of its own (decisions, "Argo's job"). 0 turns it off.
  double corridor_m = 1.5;
  double period_s = 0.1;
  double replan_period_s = 0.3;
  double horizon_s = 2.0;
  double rollout_step_s = 0.1;
  // Cost per metre of unobserved space on the path, relative to free space.
  double unknown_penalty = 1.0;
  int max_admission_checks = 40;
  // Room given to things that have moved (people), as the proxemic layer of ROS's
  // social_navigation_layers: a Gaussian cost on the gap to each one's predicted position
  // along the rollout, people_sigma_m wide, stretched ahead of it by its speed times
  // people_ahead_s, so the planner keeps out of where it is heading. Weight against the
  // carrot distance cost (metres).
  double people_cost = 1.0;
  // Drones without all-round sensing turn to keep moving things within this distance in view
  // as long as the way ahead stays in view as well.
  double watch_movers_m = 5.0;
  double people_sigma_m = 0.5;
  double people_ahead_s = 1.0;
  // Regulated pure pursuit on kinematic paths (car-like bodies): full speed is lowered in
  // proportion within this clearance of an obstacle and this distance of the goal, never
  // below the minimum (Nav2 RPP: regulated_linear_scaling_min_speed 0.25,
  // approach_velocity_scaling_dist 0.6).
  double regulate_clearance_m = 0.5;
  double regulate_approach_m = 0.8;
  double regulate_min_speed_mps = 0.15;
  // Aerial bodies: highest the base may go (SafetyConfig::ceiling_z_m); NaN for none.
  double ceiling_z_m = std::numeric_limits<double>::quiet_NaN();
};

struct Proposal {
  double inputs_stamp = 0.0;
  Twist velocity;
  bool has_target = false;
  bool path_reaches_target = false;
  // The local map shows no way to the target at all (see PathResult::exhausted).
  bool path_exhausted = false;
  bool target_blocked = false;
  // A way to the target exists in the local map, but not within the corridor around the
  // hub's legs: the hub has to route the robot another way.
  bool off_route = false;
  // How the choice went: velocities sampled, rollouts that stayed clear, supervisor checks.
  int candidates = 0;
  int clear_rollouts = 0;
  int admission_checks = 0;
};

// Tier 0 local planner: a path through the local map to the next waypoint, then the best
// sampled velocity that follows it and that the supervisor would let through.
class LocalPlanner {
 public:
  LocalPlanner(const capabilities::Capabilities& caps, const BodyVolume& body, const MotionModel& motion,
               const PlannerConfig& config);

  // `following`, the waypoint after `target` if any, sets the heading a body with a turn
  // radius should arrive at `target` with.
  // `route`, when given, is the hub's legs the path must keep near (see corridor_m).
  Proposal plan(const LocalMap& map, const Waypoint& target, const SafetySupervisor& supervisor,
                const SafetyInputs& in, const Waypoint* following = nullptr,
                const std::vector<Vec3>* route = nullptr);

  const std::vector<Vec3>& path() const { return path_; }

 private:
  struct Rollout {
    Twist velocity;
    double cost = 0.0;
  };

  // `reverse_limit_m` bounds reversing on a turn-radius-aware path (see HybridRequest).
  void replan(const LocalMap& map, const Pose4& pose, const Vec3& goal, const Waypoint* following,
              double reverse_limit_m, double now, const std::vector<Vec3>* route);
  // Drops the part of `hybrid_points_` already driven. False when the path no longer fits the
  // body's position, the goal or the map, and has to be searched again.
  bool hold_hybrid_path(const Pose4& pose, const HybridRequest& request);
  Vec3 carrot(const Pose4& pose, const Vec3& goal, double speed) const;
  // The body, keeping its heading, has a clear straight line from `pose` to `to`.
  bool clear_line(const Pose4& pose, const Vec3& to) const;
  std::vector<Twist> candidates(const Odometry& odom, const Vec3& carrot, double speed_cap) const;
  // Smallest clearance between the body and any obstacle at `pose`.
  double body_clearance(const Pose4& pose) const;
  // Rejects rollouts that come within `threshold` of an obstacle. With `contact_map` set, a
  // pose that close is judged by the supervisor's contact test instead.
  bool evaluate(const Twist& target, const Odometry& odom, const Vec3& carrot, double threshold,
                const LocalMap* contact_map, Rollout& out) const;

  const capabilities::Capabilities& caps_;
  const BodyVolume& body_;
  const MotionModel& motion_;
  PlannerConfig config_;
  Workspace ws_;
  PathSearch search_;
  HybridSearch hybrid_;
  // Bodies with a minimum turn radius that cannot strafe re-plan the grid path with Hybrid A*.
  bool use_hybrid_ = false;
  bool hybrid_path_ = false;  // path_ came from Hybrid A* this replan
  // The Hybrid A* path being followed, kept across replans while it stays free and close.
  std::vector<HybridPathPoint> hybrid_points_;
  double hybrid_planned_at_ = -std::numeric_limits<double>::infinity();
  std::vector<Vec3> path_;
  // Per path point: reached by driving backwards. Only Hybrid A* paths reverse.
  std::vector<uint8_t> path_reverse_;
  bool path_reaches_ = false;
  bool path_exhausted_ = false;
  bool target_blocked_ = false;
  bool off_route_ = false;
  double last_replan_ = -std::numeric_limits<double>::infinity();
  Vec3 last_goal_ = Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  std::vector<Vec2> probes_;
  double lookahead_m_ = 1.0;
  // Direction of travel in the base frame (radians from forward) that the planner turns
  // toward the path: the middle of the widest covered arc, or forward or reverse for bodies
  // that cannot strafe.
  double travel_bearing_ = 0.0;
  // The bearing used this tick: `travel_bearing_`, or forward or reverse as the current
  // stretch of a Hybrid A* path says.
  double active_bearing_ = 0.0;
  // Whether each level bearing around the body (body frame, equal steps from 0) is seen at
  // every height; empty when the profile has no coverage.
  std::vector<bool> covered_ring_;
  bool ring_covered(double bearing) const;
  // Yaw for a drone flying toward `heading`: the best-covered side along the way, turned
  // further if that keeps a nearby moving thing in view too (PANTHER).
  double watch_yaw(const Pose4& pose, double heading) const;
  // Tracked obstacles for this plan() call (SafetyInputs::movers).
  const std::vector<TrackedObstacle>* movers_ = nullptr;
  // Proxemic cost of the body at `pose`, `t` seconds ahead (PlannerConfig::people_*).
  double people_cost(const Pose4& pose, double t) const;
  bool reversing_stretch_ = false;
};

}  // namespace argus::node

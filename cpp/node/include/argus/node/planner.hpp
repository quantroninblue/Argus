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
  double period_s = 0.1;
  double replan_period_s = 0.3;
  double horizon_s = 2.0;
  double rollout_step_s = 0.1;
  // Cost per metre of unobserved space on the path, relative to free space.
  double unknown_penalty = 1.0;
  int max_admission_checks = 40;
};

struct Proposal {
  double inputs_stamp = 0.0;
  Twist velocity;
  bool has_target = false;
  bool path_reaches_target = false;
  // The local map shows no way to the target at all (see PathResult::exhausted).
  bool path_exhausted = false;
  bool target_blocked = false;
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
  Proposal plan(const LocalMap& map, const Waypoint& target, const SafetySupervisor& supervisor,
                const SafetyInputs& in, const Waypoint* following = nullptr);

  const std::vector<Vec3>& path() const { return path_; }

 private:
  struct Rollout {
    Twist velocity;
    double cost = 0.0;
  };

  // `reverse_limit_m` bounds reversing on a turn-radius-aware path (see HybridRequest).
  void replan(const LocalMap& map, const Pose4& pose, const Vec3& goal, const Waypoint* following,
              double reverse_limit_m, double now);
  // Drops the part of `hybrid_points_` already driven. False when the path no longer fits the
  // body's position, the goal or the map, and has to be searched again.
  bool hold_hybrid_path(const Pose4& pose, const HybridRequest& request);
  Vec3 carrot(const Pose4& pose, const Vec3& goal, double speed) const;
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
  bool reversing_stretch_ = false;
};

}  // namespace argus::node

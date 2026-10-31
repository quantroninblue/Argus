#pragma once

#include <limits>
#include <vector>

#include "argus/capabilities/capabilities.hpp"
#include "argus/node/body.hpp"
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

  Proposal plan(const LocalMap& map, const Waypoint& target, const SafetySupervisor& supervisor,
                const SafetyInputs& in);

  const std::vector<Vec3>& path() const { return path_; }

 private:
  struct Rollout {
    Twist velocity;
    double cost = 0.0;
  };

  void replan(const LocalMap& map, const Pose4& pose, const Vec3& goal);
  Vec3 carrot(const Pose4& pose, const Vec3& goal) const;
  std::vector<Twist> candidates(const Odometry& odom, const Vec3& carrot, double speed_cap) const;
  // Smallest clearance between the body and any obstacle at `pose`.
  double body_clearance(const Pose4& pose) const;
  bool evaluate(const Twist& target, const Odometry& odom, const Vec3& carrot, double threshold,
                Rollout& out) const;

  const capabilities::Capabilities& caps_;
  const BodyVolume& body_;
  const MotionModel& motion_;
  PlannerConfig config_;
  Workspace ws_;
  PathSearch search_;
  std::vector<Vec3> path_;
  bool path_reaches_ = false;
  double last_replan_ = -std::numeric_limits<double>::infinity();
  Vec3 last_goal_ = Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  std::vector<Vec2> probes_;
  double lookahead_m_ = 1.0;
  // Direction of travel in the base frame (radians from forward) that the planner turns
  // toward the path: the middle of the widest covered arc, or forward or reverse for bodies
  // that cannot strafe.
  double travel_bearing_ = 0.0;
};

}  // namespace argus::node

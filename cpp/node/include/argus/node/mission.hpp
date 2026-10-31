#pragma once

#include <string>
#include <vector>

#include "argus/node/types.hpp"

namespace argus::node {

// The plan Argo is executing: ordered waypoints with an expiry. Owns which waypoint is next.
class Mission {
 public:
  // Refuses plans older than the current one and plans that have already expired. An empty
  // plan is accepted and leaves Argo idle.
  bool accept(const Plan& plan, double now, std::string& reason);

  // Moves past reached waypoints and drops the plan when it expires.
  void update(const Pose4& pose, bool planar, double now, std::vector<Event>& events);

  // Plans Argo makes for itself (failsafes, return home). Accepted whatever their generation,
  // which stays at the hub's last one so the hub's next plan still counts as newer.
  void override_plan(const Plan& plan);
  // Drops every waypoint after the current one.
  void keep_current_only();
  void clear() { active_ = false; }
  bool active() const { return active_; }

  // Waypoint being driven to, or nullptr when idle.
  const Waypoint* target() const;
  // The waypoint after the target, or nullptr.
  const Waypoint* after_target() const;
  uint64_t generation() const { return generation_; }

 private:
  Plan plan_;
  size_t next_ = 0;
  bool active_ = false;
  uint64_t generation_ = 0;
};

}  // namespace argus::node

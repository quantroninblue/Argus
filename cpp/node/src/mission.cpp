#include "argus/node/mission.hpp"

namespace argus::node {

bool Mission::accept(const Plan& plan, double now, std::string& reason) {
  if (plan.generation <= generation_) {
    reason = "generation " + std::to_string(plan.generation) + " is not newer than " +
             std::to_string(generation_);
    return false;
  }
  if (plan.expires <= now) {
    reason = "plan already expired";
    return false;
  }
  plan_ = plan;
  generation_ = plan.generation;
  next_ = 0;
  active_ = !plan_.waypoints.empty();
  return true;
}

void Mission::update(const Pose4& pose, bool planar, double now, std::vector<Event>& events) {
  if (!active_) {
    return;
  }
  if (now >= plan_.expires) {
    active_ = false;
    events.push_back({now, EventKind::PlanExpired, "", FailureCause::None, ""});
    return;
  }
  while (next_ < plan_.waypoints.size()) {
    const Waypoint& wp = plan_.waypoints[next_];
    Vec3 offset = wp.position - pose.position;
    if (planar) {
      offset.z() = 0.0;
    }
    if (offset.norm() > wp.tolerance_m) {
      break;
    }
    events.push_back({now, EventKind::WaypointReached, wp.id, FailureCause::None, ""});
    ++next_;
  }
  if (next_ == plan_.waypoints.size()) {
    active_ = false;
    events.push_back({now, EventKind::PlanComplete, "", FailureCause::None, ""});
  }
}

void Mission::override_plan(const Plan& plan) {
  plan_ = plan;
  next_ = 0;
  active_ = !plan_.waypoints.empty();
}

void Mission::keep_current_only() {
  if (!active_) {
    return;
  }
  plan_.waypoints = {plan_.waypoints[next_]};
  next_ = 0;
}

const Waypoint* Mission::target() const {
  return active_ ? &plan_.waypoints[next_] : nullptr;
}

const Waypoint* Mission::after_target() const {
  return active_ && next_ + 1 < plan_.waypoints.size() ? &plan_.waypoints[next_ + 1] : nullptr;
}

}  // namespace argus::node

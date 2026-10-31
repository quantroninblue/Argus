#include "argus/node/node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "argus/node/adapter.hpp"

namespace argus::node {

using capabilities::MotionSpace;

namespace {

LocalMapConfig map_config(const capabilities::Profile& profile, const capabilities::Capabilities& caps,
                          const NodeConfig& config) {
  LocalMapConfig m = config.map;
  if (m.half_width_m > 0.0) {
    return m;
  }
  if (caps.motion.space == MotionSpace::Planar) {
    const double ground = profile.geometry.ground_z_m.value_or(0.0);
    m.half_width_m = 8.0;
    m.z_below_m = 0.3 - ground;
    m.z_above_m = caps.collision.aabb_max.z() + caps.collision.padding_m + 0.3;
  } else {
    m.half_width_m = 6.0;
    m.z_below_m = 3.0;
    m.z_above_m = 3.0;
  }
  return m;
}

FailureCause hard_cause(uint32_t flags) {
  if (flags & kLocalization) return FailureCause::LocalizationInvalid;
  if (flags & kDeadline) return FailureCause::ComputeDeadlineMiss;
  if (flags & kStaleSensor) return FailureCause::SensorInvalid;
  return FailureCause::None;
}

// Why the robot is not getting closer. Only a dead end may change the hub's exploration memory,
// so it is reported only when the local map itself shows no way to the waypoint.
FailureCause cause_of(uint32_t flags, const Proposal& proposal) {
  const FailureCause hard = hard_cause(flags);
  if (hard != FailureCause::None) return hard;
  if (proposal.target_blocked) return FailureCause::TemporaryObstruction;
  if (proposal.path_exhausted) return FailureCause::DeadEndObserved;
  if (flags & kBlocked) return FailureCause::TemporaryObstruction;
  return FailureCause::ControllerNoProgress;
}

std::string action_name(capabilities::LinkLostAction a) { return capabilities::to_string(a); }

}  // namespace

Node::Node(capabilities::Profile profile, NodeConfig config)
    : profile_(std::move(profile)),
      caps_(capabilities::derive_capabilities(profile_)),
      config_(config),
      ground_z_(profile_.geometry.ground_z_m.value_or(0.0)),
      map_(map_config(profile_, caps_, config)) {
  if (!interface_supported(caps_.control.interface)) {
    throw std::invalid_argument("control interface '" + capabilities::to_string(caps_.control.interface) +
                                "' is not supported yet");
  }
  control_period_ = caps_.control.rate_hz > 0.0 ? 1.0 / caps_.control.rate_hz : 0.02;
  config_.planner.period_s = std::max(config_.planner.period_s, control_period_);

  for (const auto& spec : profile_.sensors) {
    if (!capabilities::is_range_sensor(spec.kind)) {
      continue;
    }
    RangeSensor s;
    s.spec = &spec;
    s.mount = profile_.geometry.frames.at(spec.frame).transform();
    s.pattern = make_ray_pattern(spec, config_.rays);
    s.self_hits = self_hit_distances(profile_.geometry.shapes, s.mount, s.pattern, spec.range_m->max);
    sensors_[spec.id] = std::move(s);
    clock_.max_age[spec.id] = spec.max_age_s;
  }

  body_ = std::make_unique<BodyVolume>(caps_, ground_z_, config_.min_obstacle_height_m,
                                       config_.safety.max_unknown_fraction);
  motion_ = std::make_unique<MotionModel>(caps_.motion);
  supervisor_ = std::make_unique<SafetySupervisor>(caps_, *body_, *motion_, config_.safety, control_period_,
                                                   config_.planner.period_s, clock_.max_age);
  planner_ = std::make_unique<LocalPlanner>(caps_, *body_, *motion_, config_.planner);
}

const Node::RangeSensor& Node::range_sensor(const std::string& id) const {
  const auto it = sensors_.find(id);
  if (it == sensors_.end()) {
    throw std::invalid_argument("no range sensor '" + id + "'");
  }
  return it->second;
}

const RayPattern& Node::ray_pattern(const std::string& id) const { return range_sensor(id).pattern; }

const std::vector<double>& Node::self_hits(const std::string& id) const { return range_sensor(id).self_hits; }

Eigen::Isometry3d Node::sensor_mount(const std::string& id) const { return range_sensor(id).mount; }

std::vector<std::string> Node::range_sensor_ids() const {
  std::vector<std::string> ids;
  for (const auto& [id, s] : sensors_) {
    ids.push_back(id);
  }
  return ids;
}

void Node::on_odometry(const Odometry& odom) {
  if (have_odom_ && odom.stamp < odom_.stamp) {
    return;
  }
  odom_ = odom;
  have_odom_ = true;
  history_.push_back(odom);
  while (!history_.empty() && history_.front().stamp < odom.stamp - config_.odometry_history_s) {
    history_.pop_front();
  }
  map_.recenter(odom.pose.position);
  if (crumbs_.empty() || (crumbs_.back() - odom.pose.position).norm() >= config_.breadcrumb_spacing_m) {
    crumbs_.push_back(odom.pose.position);
    if (crumbs_.size() > config_.max_breadcrumbs) {
      // Thin the trail evenly, keeping home and the newest point.
      std::vector<Vec3> thinned;
      for (size_t i = 0; i < crumbs_.size(); i += 2) thinned.push_back(crumbs_[i]);
      if (!(thinned.back() - crumbs_.back()).isZero()) thinned.push_back(crumbs_.back());
      crumbs_.swap(thinned);
    }
  }
}

bool Node::pose_at(double stamp, Pose4& pose) const {
  if (history_.empty() || stamp < history_.front().stamp - 1e-9) {
    return false;
  }
  const Odometry& last = history_.back();
  if (stamp >= last.stamp) {
    const double dt = stamp - last.stamp;
    if (dt > 0.1) {
      return false;
    }
    pose = MotionModel::integrate(last.pose, last.twist, dt);
    return true;
  }
  const auto after = std::lower_bound(history_.begin(), history_.end(), stamp,
                                      [](const Odometry& o, double t) { return o.stamp < t; });
  if (after == history_.begin()) {
    pose = after->pose;
    return true;
  }
  const Odometry& a = *(after - 1);
  const Odometry& b = *after;
  const double span = b.stamp - a.stamp;
  const double f = span > 0.0 ? (stamp - a.stamp) / span : 0.0;
  pose.position = a.pose.position + f * (b.pose.position - a.pose.position);
  pose.yaw = wrap_angle(a.pose.yaw + f * wrap_angle(b.pose.yaw - a.pose.yaw));
  return true;
}

bool Node::on_range_frame(const RangeFrame& frame) {
  const auto it = sensors_.find(frame.sensor_id);
  if (it == sensors_.end()) {
    return false;
  }
  const RangeSensor& s = it->second;
  if (frame.ranges.size() != s.pattern.size()) {
    return false;
  }
  Pose4 pose;
  if (!pose_at(frame.stamp, pose)) {
    return false;
  }
  const Eigen::Isometry3d to_odom = pose.transform() * s.mount;
  const Vec3 origin = to_odom.translation();
  const double min_range = s.spec->range_m->min;
  const double max_range = s.spec->range_m->max;
  const bool planar = caps_.motion.space == MotionSpace::Planar;
  const double ground = pose.position.z() + ground_z_ + config_.min_obstacle_height_m;

  std::vector<Vec3> ends;
  std::vector<bool> hits;
  ends.reserve(frame.ranges.size());
  hits.reserve(frame.ranges.size());
  for (size_t i = 0; i < frame.ranges.size(); ++i) {
    if (std::isfinite(s.self_hits[i])) {
      continue;  // this ray looks at the robot itself
    }
    const double r = frame.ranges[i];
    if (std::isnan(r) || r < min_range) {
      continue;
    }
    const Vec3 dir = to_odom.linear() * s.pattern.directions[i];
    bool hit = std::isfinite(r) && r <= max_range;
    if (!hit && !config_.no_return_is_free) {
      continue;
    }
    const Vec3 end = origin + dir * (hit ? r : max_range);
    if (hit && planar && end.z() < ground) {
      hit = false;
    }
    ends.push_back(end);
    hits.push_back(hit);
  }
  map_.integrate(origin, ends, hits);
  double& last = clock_.last_frame[frame.sensor_id];
  last = std::max(last, frame.stamp);
  return true;
}

bool Node::set_plan(const Plan& plan, double now) {
  std::string reason;
  if (!caps_.safety_ready) {
    reason = "profile has unreviewed LLM draft fields";
  } else if (failsafe_active_) {
    reason = "failsafe active";
  } else if (mission_.accept(plan, now, reason)) {
    pending_.push_back({now, EventKind::PlanAccepted, "", FailureCause::None, ""});
    return true;
  }
  pending_.push_back({now, EventKind::PlanRejected, "", FailureCause::None, reason});
  return false;
}

void Node::watch_progress(const Waypoint* target, uint32_t flags, double now, std::vector<Event>& events) {
  if (!target) {
    progress_target_.clear();
    return;
  }
  Vec3 offset = target->position - odom_.pose.position;
  if (caps_.motion.space == MotionSpace::Planar) {
    offset.z() = 0.0;
  }
  const double distance = offset.norm();
  const std::string key = std::to_string(mission_.generation()) + "/" + target->id;
  if (key != progress_target_) {
    progress_target_ = key;
    best_distance_ = distance;
    last_progress_ = now;
  } else if (distance < best_distance_ - 0.1) {
    best_distance_ = distance;
    last_progress_ = now;
  } else if (now - last_progress_ > config_.no_progress_s) {
    events.push_back({now, EventKind::Blocked, target->id, cause_of(flags, proposal_), "no progress"});
    last_progress_ = now;
  }
}

void Node::watch_dead_end(const Waypoint* target, double now, std::vector<Event>& events) {
  const bool exhausted = target && proposal_.has_target && proposal_.path_exhausted && !proposal_.target_blocked;
  const std::string key = target ? std::to_string(mission_.generation()) + "/" + target->id : "";
  if (key != dead_end_target_) {
    dead_end_target_ = key;
    dead_end_reported_ = false;
    exhausted_since_ = -1.0;
  }
  if (!exhausted) {
    exhausted_since_ = -1.0;
    return;
  }
  if (exhausted_since_ < 0.0) {
    exhausted_since_ = now;
  }
  if (!dead_end_reported_ && now - exhausted_since_ >= config_.dead_end_s) {
    events.push_back({now, EventKind::Blocked, target->id, FailureCause::DeadEndObserved,
                      "no way through the local map"});
    dead_end_reported_ = true;
  }
}

void Node::cancel_plan(uint64_t generation, bool home, double now) {
  if (generation < mission_.generation() || failsafe_active_) {
    return;
  }
  mission_.clear();
  if (home) {
    return_home(now);
  }
}

void Node::set_paused(bool paused, double) { paused_ = paused; }

Plan Node::home_plan() const {
  Plan plan;
  plan.generation = mission_.generation();
  plan.expires = std::numeric_limits<double>::infinity();
  if (crumbs_.empty() || !have_odom_) {
    return plan;
  }
  // Walk the trail backwards, cutting out loops: when a crumb comes close to one already on the
  // way back, everything after that one is a detour.
  const double near = config_.breadcrumb_spacing_m;
  std::vector<Vec3> back{odom_.pose.position};
  for (auto it = crumbs_.rbegin(); it != crumbs_.rend(); ++it) {
    size_t hit = back.size();
    for (size_t i = 0; i + 1 < back.size(); ++i) {
      if ((back[i] - *it).norm() < near) {
        hit = i;
        break;
      }
    }
    if (hit < back.size()) {
      back.resize(hit + 1);
    } else {
      back.push_back(*it);
    }
  }
  back.push_back(crumbs_.front());
  double since = 0.0;
  for (size_t i = 1; i < back.size(); ++i) {
    since += (back[i] - back[i - 1]).norm();
    const bool last = i + 1 == back.size();
    if (since >= 1.0 || last) {
      Waypoint wp;
      wp.id = "home-" + std::to_string(plan.waypoints.size());
      wp.position = back[i];
      wp.tolerance_m = last ? 0.25 : 0.4;
      plan.waypoints.push_back(wp);
      since = 0.0;
    }
  }
  return plan;
}

Plan Node::land_plan() const {
  Plan plan;
  plan.generation = mission_.generation();
  plan.expires = std::numeric_limits<double>::infinity();
  if (!have_odom_) {
    return plan;
  }
  // Highest observed obstacle below the base: the floor to settle above. With no floor in
  // view, stay where we are rather than descend blind.
  const Vec3 p = odom_.pose.position;
  const Index3 top = map_.index_of(p);
  for (int z = top.z() - 1; z >= map_.window_min().z(); --z) {
    if (map_.cell(Index3(top.x(), top.y(), z)) == Cell::Occupied) {
      const double floor_z = (z + 1) * map_.resolution();
      Waypoint wp;
      wp.id = "land";
      wp.position = Vec3(p.x(), p.y(), std::min(p.z(), floor_z + body_->radius_m() + 0.15));
      wp.tolerance_m = 0.15;
      plan.waypoints.push_back(wp);
      break;
    }
  }
  return plan;
}

void Node::return_home(double now) {
  if (failsafe_active_) {
    return;
  }
  mission_.override_plan(home_plan());
  pending_.push_back({now, EventKind::PlanAccepted, "", FailureCause::None, "return home"});
}

void Node::enter_failsafe(capabilities::LinkLostAction action, double now) {
  if (failsafe_active_) {
    return;
  }
  failsafe_active_ = true;
  failsafe_action_ = action;
  paused_ = false;
  using capabilities::LinkLostAction;
  switch (action) {
    case LinkLostAction::Stop:
    case LinkLostAction::Hold:
      mission_.clear();
      break;
    case LinkLostAction::FinishLegThenStop:
      mission_.keep_current_only();
      break;
    case LinkLostAction::ReturnHome:
      mission_.override_plan(home_plan());
      break;
    case LinkLostAction::Land:
      mission_.override_plan(land_plan());
      break;
  }
  pending_.push_back({now, EventKind::FailsafeEntered, "", FailureCause::LinkLost, action_name(action)});
  if (!mission_.active()) {
    pending_.push_back({now, EventKind::FailsafeDone, "", FailureCause::None, action_name(action)});
  }
}

void Node::clear_failsafe(double now) {
  if (!failsafe_active_) {
    return;
  }
  failsafe_active_ = false;
  mission_.clear();
  pending_.push_back({now, EventKind::FailsafeCleared, "", FailureCause::None, action_name(failsafe_action_)});
}

TickResult Node::tick(double now) {
  TickResult result;
  result.events.swap(pending_);
  if (!have_odom_) {
    result.safety_flags = kLocalization;
    result.command.stamp = now;
    return result;
  }

  const bool planar = caps_.motion.space == MotionSpace::Planar;
  if (failsafe_active_) {
    // The failsafe's own plan: report only that it finished, not its waypoints.
    std::vector<Event> internal;
    const bool was_active = mission_.active();
    mission_.update(odom_.pose, planar, now, internal);
    if (was_active && !mission_.active()) {
      result.events.push_back({now, EventKind::FailsafeDone, "", FailureCause::None, action_name(failsafe_action_)});
    }
  } else {
    mission_.update(odom_.pose, planar, now, result.events);
  }
  const Waypoint* active = paused_ ? nullptr : mission_.target();
  Waypoint capped;
  if (active && speed_cap_ > 0.0) {
    capped = *active;
    capped.speed_cap_mps = capped.speed_cap_mps > 0.0 ? std::min(capped.speed_cap_mps, speed_cap_) : speed_cap_;
    active = &capped;
  }
  const Waypoint* target = active;
  const SafetyInputs in{odom_, map_, clock_, now};

  // Out of creep and still nothing it can see to drive on: go back to where the creep began.
  // The planner turns the body so it sees the way back, which is space it has just crossed.
  Waypoint retrace;
  if (target) {
    Vec3 left = creep_start_ - odom_.pose.position;
    if (planar) {
      left.z() = 0.0;
    }
    if (retracing_ && (target->id != retrace_for_ || left.norm() < 0.2 || supervisor_->creep_available())) {
      retracing_ = false;
    }
    if (!retracing_ && supervisor_->creep_allowed() && !supervisor_->creep_available() &&
        proposal_.has_target && proposal_.velocity.linear.norm() < 1e-6 && left.norm() >= 0.2) {
      retracing_ = true;
      retrace_for_ = target->id;
    }
    if (retracing_) {
      retrace = *target;
      retrace.position = creep_start_;
      retrace.tolerance_m = 0.2;
      target = &retrace;
    }
  } else {
    retracing_ = false;
  }

  if (velocity_requested(now)) {
    // A setup motion test: the requested velocity instead of the planner's, checked below by
    // the supervisor like any other.
    proposal_ = Proposal{};
    proposal_.inputs_stamp = now;
    proposal_.velocity = requested_;
  } else if (!target) {
    proposal_ = Proposal{};
    proposal_.inputs_stamp = now;
  } else if (now - last_plan_time_ >= config_.planner.period_s - 1e-9 || !proposal_.has_target) {
    const Waypoint* following = target == mission_.target() ? mission_.after_target() : nullptr;
    proposal_ = planner_->plan(map_, *target, *supervisor_, in, following);
    last_plan_time_ = now;
  }

  const SafetySupervisor::Verdict verdict = supervisor_->check(proposal_.velocity, proposal_.inputs_stamp, in);
  if ((verdict.flags & kCreep) && !(last_flags_ & kCreep) &&
      supervisor_->creep_left_m() >= supervisor_->creep_budget_m() - 1e-9) {
    creep_start_ = odom_.pose.position;
  }
  result.safety_flags = verdict.flags | (target ? 0u : static_cast<uint32_t>(kNoTarget));
  result.proposed = proposal_.velocity;
  result.proposal = proposal_;
  result.command = make_command(caps_.control.interface, caps_.motion, odom_, verdict.velocity, now,
                                control_period_ + caps_.motion.latency_s);
  std::vector<Event> step_events;
  if (target) {
    result.target_id = target->id;
    const uint32_t hard = kStaleSensor | kLocalization | kDeadline;
    const uint32_t raised = verdict.flags & hard & ~last_flags_;
    if (raised) {
      step_events.push_back({now, EventKind::Blocked, target->id, hard_cause(raised), ""});
    }
  }
  watch_progress(target, verdict.flags, now, step_events);
  watch_dead_end(target, now, step_events);
  if (!failsafe_active_) {
    result.events.insert(result.events.end(), step_events.begin(), step_events.end());
  }
  last_flags_ = verdict.flags;
  return result;
}

}  // namespace argus::node

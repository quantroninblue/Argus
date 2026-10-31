#include "argus/node/node.hpp"

#include <algorithm>
#include <cmath>
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

FailureCause cause_of(uint32_t flags) {
  if (flags & kLocalization) return FailureCause::LocalizationInvalid;
  if (flags & kDeadline) return FailureCause::ComputeDeadlineMiss;
  if (flags & kStaleSensor) return FailureCause::SensorInvalid;
  if (flags & kBlocked) return FailureCause::TemporaryObstruction;
  return FailureCause::ControllerNoProgress;
}

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
    events.push_back({now, EventKind::Blocked, target->id, cause_of(flags), "no progress"});
    last_progress_ = now;
  }
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
  mission_.update(odom_.pose, planar, now, result.events);
  const Waypoint* target = mission_.target();
  const SafetyInputs in{odom_, map_, clock_, now};

  if (!target) {
    proposal_ = Proposal{};
    proposal_.inputs_stamp = now;
  } else if (now - last_plan_time_ >= config_.planner.period_s - 1e-9 || !proposal_.has_target) {
    proposal_ = planner_->plan(map_, *target, *supervisor_, in);
    last_plan_time_ = now;
  }

  const SafetySupervisor::Verdict verdict = supervisor_->check(proposal_.velocity, proposal_.inputs_stamp, in);
  result.safety_flags = verdict.flags | (target ? 0u : static_cast<uint32_t>(kNoTarget));
  result.proposed = proposal_.velocity;
  result.proposal = proposal_;
  result.command = make_command(caps_.control.interface, caps_.motion, odom_, verdict.velocity, now,
                                control_period_ + caps_.motion.latency_s);
  if (target) {
    result.target_id = target->id;
    const uint32_t hard = kStaleSensor | kLocalization | kDeadline;
    const uint32_t raised = verdict.flags & hard & ~last_flags_;
    if (raised) {
      result.events.push_back({now, EventKind::Blocked, target->id, cause_of(raised), ""});
    }
  }
  watch_progress(target, verdict.flags, now, result.events);
  last_flags_ = verdict.flags;
  return result;
}

}  // namespace argus::node

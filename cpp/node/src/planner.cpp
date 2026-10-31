#include "argus/node/planner.hpp"

#include <algorithm>
#include <cmath>

namespace argus::node {

using capabilities::MotionSpace;

namespace {

constexpr double kComfortMargin = 0.3;
// Kinematic paths are planned with a turn radius this much wider than the profile's, so the
// tracker has steering left to correct errors: real turns come out wider than declared.
constexpr double kTurnRadiusMargin = 1.2;
// A held kinematic path is searched again once the body is this far from it, and at least
// this often (Nav2 replans at 1 Hz by default), so a body that turns wider than the plan
// gets a new path from where it really is.
constexpr double kHoldDistance = 0.2;
constexpr double kHoldPeriod = 1.0;
// Kinematic paths are tracked with a lookahead proportional to speed (Adaptive Pure Pursuit;
// Macenski et al. 2023 use 1 s, 0.25 to 1.2 m): a long fixed one cuts corners into walls.
constexpr double kLookaheadTime = 1.0;
constexpr double kMinLookahead = 0.4;
constexpr double kMaxLookahead = 1.0;

// Centre of the longest run of covered bearings, treating the bins as a circle, preferring the
// run that contains forward among equally long ones. 0 (forward) when nothing or everything is
// covered.
double widest_covered_bearing(const std::vector<const capabilities::CoverageBin*>& ring) {
  const int n = static_cast<int>(ring.size());
  auto covered = [&](int i) {
    const auto* b = ring[((i % n) + n) % n];
    return b->full_height_reach_m > 0.0;
  };
  int best_start = 0;
  int best_len = 0;
  for (int i = 0; i < n; ++i) {
    if (!covered(i) || covered(i - 1)) {
      continue;  // runs start right after an uncovered bin
    }
    int len = 0;
    while (len < n && covered(i + len)) ++len;
    const bool has_forward = (n - i) % n < len;
    if (len > best_len || (len == best_len && has_forward)) {
      best_len = len;
      best_start = i;
    }
  }
  if (best_len == 0) {
    return 0.0;
  }
  return wrap_angle((best_start + 0.5 * (best_len - 1)) * 2.0 * M_PI / n);
}

Twist cap_speed(Twist v, double cap) {
  const double speed = v.linear.norm();
  if (cap > 0.0 && speed > cap) {
    const double s = cap / speed;
    v.linear *= s;
    v.yaw_rate *= std::min(1.0, s * 2.0);
  }
  return v;
}

}  // namespace

LocalPlanner::LocalPlanner(const capabilities::Capabilities& caps, const BodyVolume& body,
                           const MotionModel& motion, const PlannerConfig& config)
    : caps_(caps), body_(body), motion_(motion), config_(config) {
  // Probe points around the padded footprint, no further apart than half a cell.
  const auto& fp = body_.footprint();
  for (size_t i = 0; i < fp.size(); ++i) {
    const Vec2 a = fp[i];
    const Vec2 b = fp[(i + 1) % fp.size()];
    const int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / 0.05)));
    for (int k = 0; k < n; ++k) {
      probes_.push_back(a + (b - a) * (static_cast<double>(k) / n));
    }
  }
  const auto& m = caps_.motion;
  const double top = std::max({m.x.max_mps, -m.x.min_mps, m.y.max_mps, m.z.max_mps});
  lookahead_m_ = std::clamp(1.5 * top, 0.8, 3.0) + body_.radius_m();

  // Bearings around the body at its own height: the planar bins, or the level row for aerial.
  std::vector<const capabilities::CoverageBin*> ring;
  for (const auto& b : caps_.sensing.bins) {
    if (m.space == MotionSpace::Planar || b.elevation_deg == 0.0) {
      ring.push_back(&b);
    }
  }
  if (ring.empty()) {
    return;
  }
  auto covered_at = [&](double bearing) {
    const int n = static_cast<int>(ring.size());
    double deg = bearing * 180.0 / M_PI;
    if (deg < 0.0) deg += 360.0;
    const auto* b = ring[static_cast<int>(std::lround(deg / (360.0 / n))) % n];
    return b->full_height_reach_m > 0.0;
  };
  use_hybrid_ = m.space == MotionSpace::Planar && !m.can_strafe && m.min_turn_radius_m > 0.0;
  if (m.space == MotionSpace::Planar && !m.can_strafe) {
    const bool reverse = m.can_reverse && m.x.min_mps < 0.0;
    travel_bearing_ = !covered_at(0.0) && reverse && covered_at(M_PI) ? M_PI : 0.0;
  } else {
    travel_bearing_ = widest_covered_bearing(ring);
  }
  active_bearing_ = travel_bearing_;
}

double LocalPlanner::body_clearance(const Pose4& pose) const {
  if (!body_.planar()) {
    return ws_.clearance_at(pose.position) - body_.radius_m();
  }
  double best = Workspace::kFar;
  for (const Vec2& p : probes_) {
    best = std::min(best, ws_.clearance_at(pose.to_odom(Vec3(p.x(), p.y(), 0.0))));
  }
  return best;
}

void LocalPlanner::replan(const LocalMap& map, const Pose4& pose, const Vec3& goal, const Waypoint* following,
                          double reverse_limit_m, double now) {
  ws_.build(map, body_, pose);
  PathRequest req;
  req.start = pose.position;
  req.goal = goal;
  req.unknown_penalty = config_.unknown_penalty;
  if (body_.planar()) {
    req.min_clearance_m = std::max(0.0, body_.inscribed_radius_m() - 0.5 * ws_.resolution());
    req.comfort_clearance_m = body_.radius_m() + kComfortMargin;
  } else {
    req.min_clearance_m = body_.radius_m();
    req.comfort_clearance_m = body_.radius_m() + kComfortMargin;
  }
  const PathResult result = search_.find(ws_, req);
  path_ = result.points;
  path_reverse_.assign(path_.size(), 0);
  hybrid_path_ = false;
  if (use_hybrid_ && result.reached && path_.size() >= 2) {
    HybridRequest hr;
    hr.start = pose.position;
    hr.start_yaw = pose.yaw;
    hr.goal = path_.back();
    hr.goal_tolerance_m = 1.5 * ws_.resolution();
    // Arrive lined up with the leg after this waypoint (route-aligned end states, as lattice
    // planners for cars do), unless the goal was clipped to the window.
    if (following != nullptr && (path_.back() - goal).head<2>().norm() < 2.0 * ws_.resolution()) {
      const Vec3 leg = following->position - goal;
      if (leg.head<2>().norm() > 0.3) {
        hr.goal_heading_set = true;
        hr.goal_yaw = std::atan2(leg.y(), leg.x());
      }
    }
    hr.turn_radius_m = kTurnRadiusMargin * caps_.motion.min_turn_radius_m;
    hr.reverse_limit_m = caps_.motion.can_reverse && caps_.motion.x.min_mps < 0.0 ? reverse_limit_m : 0.0;
    hr.probes = &probes_;
    hr.outer_radius_m = body_.radius_m() + ws_.resolution();
    hr.comfort_clearance_m = req.comfort_clearance_m;
    hr.unknown_penalty = req.unknown_penalty;
    if (now - hybrid_planned_at_ >= kHoldPeriod || !hold_hybrid_path(pose, hr)) {
      hybrid_planned_at_ = now;
      const HybridResult hy = hybrid_.find(ws_, hr);
      hybrid_points_ = hy.reached && hy.points.size() >= 2 ? hy.points : std::vector<HybridPathPoint>{};
    }
    // Without a kinematic path the grid path still leads the way, as before.
    if (!hybrid_points_.empty()) {
      path_.clear();
      path_reverse_.clear();
      for (const auto& p : hybrid_points_) {
        path_.push_back(p.position);
        path_reverse_.push_back(p.reverse ? 1 : 0);
      }
      hybrid_path_ = true;
    }
  } else {
    hybrid_points_.clear();
  }
  path_reaches_ = result.reached;
  path_exhausted_ = result.exhausted;
  target_blocked_ = result.goal_blocked;
}

bool LocalPlanner::hold_hybrid_path(const Pose4& pose, const HybridRequest& request) {
  auto& pts = hybrid_points_;
  if (pts.size() < 2 || (pts.back().position - request.goal).head<2>().norm() > 2.0 * ws_.resolution()) {
    return false;
  }
  if (request.goal_heading_set &&
      std::abs(wrap_angle(pts.back().yaw - request.goal_yaw)) > request.goal_heading_tolerance) {
    return false;
  }
  // Nearest point on the stretch being driven (up to the first change of direction).
  size_t end = pts.size();
  for (size_t i = 2; i < pts.size(); ++i) {
    if (pts[i].reverse != pts[1].reverse) {
      end = i;
      break;
    }
  }
  size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < end; ++i) {
    const double d = (pts[i].position - pose.position).head<2>().norm();
    if (d < best) {
      best = d;
      nearest = i;
    }
  }
  if (best > kHoldDistance) {
    return false;
  }
  // At the end of a stretch with another after it, the next one starts here.
  if (end < pts.size() && nearest + 1 >= end && best < 2.0 * ws_.resolution()) {
    nearest = end - 1;
  }
  pts.erase(pts.begin(), pts.begin() + static_cast<long>(nearest));
  if (pts.size() < 2) {
    return false;
  }
  pts.front().position = pose.position;
  pts.front().yaw = pose.yaw;
  return hybrid_.path_free(ws_, request, std::vector<HybridPathPoint>(pts.begin() + 1, pts.end()));
}

Vec3 LocalPlanner::carrot(const Pose4& pose, const Vec3& goal, double speed) const {
  if (path_.size() < 2) {
    return goal;
  }
  // A path that changes direction is followed up to the first change; the next replan, from
  // there, starts with the next stretch.
  size_t end = path_.size();
  for (size_t i = 2; i < path_.size(); ++i) {
    if (path_reverse_[i] != path_reverse_[1]) {
      end = i;
      break;
    }
  }
  size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < end; ++i) {
    const double d = (path_[i] - pose.position).norm();
    if (d < best) {
      best = d;
      nearest = i;
    }
  }
  const double lookahead =
      hybrid_path_ ? std::clamp(kLookaheadTime * speed, kMinLookahead, kMaxLookahead) : lookahead_m_;
  double travelled = 0.0;
  for (size_t i = nearest; i + 1 < end; ++i) {
    const double seg = (path_[i + 1] - path_[i]).norm();
    if (travelled + seg >= lookahead) {
      return path_[i] + (path_[i + 1] - path_[i]) * ((lookahead - travelled) / seg);
    }
    travelled += seg;
  }
  return path_[end - 1];
}

std::vector<Twist> LocalPlanner::candidates(const Odometry& odom, const Vec3& carrot, double cap) const {
  const auto& m = caps_.motion;
  std::vector<Twist> out;
  const double fractions[] = {-1.0, -0.6, -0.3, -0.12, 0.0, 0.12, 0.3, 0.6, 1.0};

  if (m.space == MotionSpace::Planar) {
    std::vector<double> xs = {0.0};
    for (double f : {1.0, 0.7, 0.45, 0.25, 0.1}) xs.push_back(f * m.x.max_mps);
    // Reversing is offered slowly, unless the body travels backwards.
    if (m.can_reverse && m.x.min_mps < 0.0) {
      if (std::cos(active_bearing_) < 0.0) {
        for (double f : {1.0, 0.7, 0.45, 0.25, 0.1}) xs.push_back(f * m.x.min_mps);
      } else {
        for (double f : {0.3, 0.15}) xs.push_back(f * m.x.min_mps);
      }
    }
    std::vector<double> ys = {0.0};
    if (m.can_strafe) {
      for (double f : {1.0, 0.5}) {
        ys.push_back(f * m.y.max_mps);
        ys.push_back(f * m.y.min_mps);
      }
    }
    for (double vx : xs) {
      for (double vy : ys) {
        for (double f : fractions) {
          Twist t;
          t.linear = Vec3(vx, vy, 0.0);
          t.yaw_rate = m.min_turn_radius_m > 0.0 ? vx * f / m.min_turn_radius_m : f * m.yaw_rate_max_radps;
          out.push_back(motion_.clamp(cap_speed(t, cap)));
        }
      }
    }
    // On a kinematic path, the pure pursuit arc to the carrot (curvature 2y/L^2 in the base
    // frame; Coulter 1992, Macenski et al. 2023), so the path can be followed exactly.
    if (hybrid_path_) {
      const Vec3 local = Pose4{Vec3::Zero(), -odom.pose.yaw}.rotate(carrot - odom.pose.position);
      const double l2 = local.head<2>().squaredNorm();
      if (l2 > 1e-4) {
        const double k = std::clamp(2.0 * local.y() / l2, -1.0 / m.min_turn_radius_m, 1.0 / m.min_turn_radius_m);
        for (double vx : xs) {
          if (vx == 0.0 || (vx < 0.0) != reversing_stretch_) continue;
          Twist t;
          t.linear = Vec3(vx, 0.0, 0.0);
          t.yaw_rate = vx * k;
          out.push_back(motion_.clamp(cap_speed(t, cap)));
        }
      }
    }
    return out;
  }

  // Aerial: directions around the one toward the carrot, at several speeds, turning so the
  // sensors look where the body goes.
  const Pose4& pose = odom.pose;
  const Vec3 to = carrot - pose.position;
  const double dist = to.norm();
  const double heading_to = std::atan2(to.y(), to.x());
  // Turn so the body's best-covered side, not necessarily its nose, faces `heading`.
  auto yaw_rate_toward = [&](double heading) {
    return std::clamp(2.0 * wrap_angle(heading - active_bearing_ - pose.yaw), -m.yaw_rate_max_radps,
                      m.yaw_rate_max_radps);
  };
  for (double w : {yaw_rate_toward(heading_to), 0.0}) {
    Twist t;
    t.yaw_rate = w;
    out.push_back(motion_.clamp(t));
  }
  if (dist < 1e-3) {
    return out;
  }
  const double el_to = std::atan2(to.z(), to.head<2>().norm());
  const double yaw_offsets[] = {0.0, 0.35, -0.35, 0.8, -0.8, 1.57, -1.57};
  const double el_offsets[] = {0.0, 0.5, -0.5};
  std::vector<Vec3> dirs = {Vec3::UnitZ(), -Vec3::UnitZ()};
  for (double dy : yaw_offsets) {
    const double h = heading_to + dy;
    for (double de : el_offsets) {
      const double e = std::clamp(el_to + de, -1.4, 1.4);
      dirs.emplace_back(std::cos(e) * std::cos(h), std::cos(e) * std::sin(h), std::sin(e));
    }
    // Level flight too: bodies with planar scanners may see nothing above or below the level
    // row, and would otherwise have no covered direction while the carrot is higher or lower.
    if (std::abs(el_to) > 0.05) {
      dirs.emplace_back(std::cos(h), std::sin(h), 0.0);
    }
  }
  for (const Vec3& dir : dirs) {
    const Vec3 body_dir = Pose4{Vec3::Zero(), -pose.yaw}.rotate(dir);
    // Fastest speed along this direction that keeps every axis inside its limits.
    double top = std::numeric_limits<double>::infinity();
    const capabilities::AxisCapability* axes[] = {&m.x, &m.y, &m.z};
    for (int a = 0; a < 3; ++a) {
      if (body_dir[a] > 1e-6) top = std::min(top, axes[a]->max_mps / body_dir[a]);
      if (body_dir[a] < -1e-6) top = std::min(top, axes[a]->min_mps / body_dir[a]);
    }
    if (!std::isfinite(top) || top <= 0.0) {
      continue;
    }
    const double horizontal = dir.head<2>().norm();
    const double w = horizontal > 0.3 ? yaw_rate_toward(std::atan2(dir.y(), dir.x())) : 0.0;
    // Fixed fractions of top speed, plus one that covers the remaining distance in about the
    // planning horizon, for the last metre or so.
    const double settle = dist / config_.horizon_s;
    for (double speed : {top, 0.6 * top, 0.3 * top, 0.12 * top, std::min(top, settle)}) {
      Twist t;
      t.linear = body_dir * speed;
      t.yaw_rate = w;
      out.push_back(motion_.clamp(cap_speed(t, cap)));
    }
  }
  return out;
}

bool LocalPlanner::evaluate(const Twist& target, const Odometry& odom, const Vec3& carrot,
                            double threshold, const LocalMap* contact_map, Rollout& out) const {
  const auto& m = caps_.motion;
  const double dt = config_.rollout_step_s;
  const int steps = static_cast<int>(std::ceil(config_.horizon_s / dt));
  Pose4 pose = odom.pose;
  Twist vel = odom.twist;
  double min_clear = Workspace::kFar;
  int unknown = 0;
  // Closest approach to the carrot, so rollouts that pass through it are not penalised for
  // overshooting: the planner runs again long before the rollout ends.
  auto distance_to_carrot = [&](const Pose4& p) {
    Vec3 to = carrot - p.position;
    if (body_.planar()) {
      to.z() = 0.0;
    }
    return to.norm();
  };
  const double start_distance = distance_to_carrot(pose);
  double distance = start_distance;
  // Heading is judged against the bearing to the carrot from where the rollout starts, and by
  // its best value along the rollout, for the same reason as the distance.
  const Vec3 from_start = carrot - odom.pose.position;
  const bool judge_heading = from_start.head<2>().norm() > 0.2;
  const double wanted_yaw = std::atan2(from_start.y(), from_start.x()) - active_bearing_;
  auto heading_error = [&](const Pose4& p) { return std::abs(wrap_angle(wanted_yaw - p.yaw)); };
  double heading = heading_error(pose);
  for (int i = 0; i < steps; ++i) {
    vel = motion_.accelerate(vel, target, dt);
    pose = MotionModel::integrate(pose, vel, dt);
    const double clear = body_clearance(pose);
    if (clear < threshold &&
        (!contact_map || body_.contact(*contact_map, pose, &odom.pose) == Contact::Occupied)) {
      return false;
    }
    min_clear = std::min(min_clear, clear);
    distance = std::min(distance, distance_to_carrot(pose));
    heading = std::min(heading, heading_error(pose));
    if (ws_.state_at(pose.position) == Cell::Unknown) {
      ++unknown;
    }
  }
  // Closest approach alone makes driving away free; a rollout that ends farther from the
  // carrot than it started pays for the distance lost.
  double cost = distance + 0.5 * std::max(0.0, distance_to_carrot(pose) - start_distance);
  if (judge_heading) {
    const bool steers = m.space == MotionSpace::Planar && !m.can_strafe;
    cost += (steers ? 0.4 : 0.2) * heading;
  }
  // Comfort is wanted up to the clearance the carrot itself has; a target close to an obstacle
  // (a landing height, a waypoint by a wall) must stay reachable.
  const double comfort = std::clamp(body_clearance(Pose4{carrot, odom.pose.yaw}), 0.0, kComfortMargin);
  cost += 0.5 * std::max(0.0, comfort - min_clear) / kComfortMargin;
  cost += 0.3 * unknown / static_cast<double>(steps);
  if (target.linear.x() < 0.0 && m.space == MotionSpace::Planar && active_bearing_ == 0.0) {
    cost += 0.3;
  }
  if (reversing_stretch_ && target.linear.x() > 0.0) {
    cost += 0.3;
  }
  out.velocity = target;
  out.cost = cost;
  return true;
}

Proposal LocalPlanner::plan(const LocalMap& map, const Waypoint& target,
                            const SafetySupervisor& supervisor, const SafetyInputs& in, const Waypoint* following) {
  Proposal proposal;
  proposal.inputs_stamp = in.odom.stamp;
  proposal.has_target = true;
  const Pose4& pose = in.odom.pose;

  const bool goal_moved = !((target.position - last_goal_).norm() < 1e-6);
  if (goal_moved || in.now - last_replan_ >= config_.replan_period_s) {
    // Reversing on a kinematic path: free where the body sees behind, the creep left where it
    // does not, none where the supervisor would refuse it.
    double reverse_limit = 0.0;
    if (use_hybrid_) {
      const Coverage c = supervisor.coverage(-Vec3::UnitX(), in);
      if (c == Coverage::Covered) {
        reverse_limit = std::numeric_limits<double>::infinity();
      } else if (c == Coverage::Uncovered && supervisor.creep_available()) {
        reverse_limit = supervisor.creep_left_m();
      }
    }
    replan(map, pose, target.position, following, reverse_limit, in.now);
    last_replan_ = in.now;
    last_goal_ = target.position;
  }
  proposal.path_reaches_target = path_reaches_;
  proposal.path_exhausted = path_exhausted_;
  proposal.target_blocked = target_blocked_;

  reversing_stretch_ = path_.size() >= 2 && path_reverse_[1] != 0;
  active_bearing_ = reversing_stretch_ ? M_PI : (hybrid_path_ ? 0.0 : travel_bearing_);
  const Vec3 aim = carrot(pose, target.position, in.odom.twist.linear.norm());
  // Within a cell of an obstacle the distance field cannot tell a move that touches it from
  // one that backs away, so rollouts there are judged by the supervisor's own contact test.
  const double threshold = 0.75 * ws_.resolution();
  const bool near = body_clearance(pose) < threshold;

  // Apply the supervisor's coverage rules before scoring, so a direction it would cap to creep
  // speed or refuse cannot win on paper.
  const bool creep = supervisor.creep_available();
  std::vector<Rollout> rollouts;
  for (Twist t : candidates(in.odom, aim, target.speed_cap_mps)) {
    ++proposal.candidates;
    const double speed = t.linear.norm();
    if (speed > 1e-6) {
      const Vec3 dir = t.linear / speed;
      const Coverage c = supervisor.coverage(dir, in);
      if (c == Coverage::Covered) {
        t = motion_.clamp(cap_speed(t, supervisor.sight_speed_limit(dir)));
      } else if (c == Coverage::Uncovered && creep) {
        t = motion_.clamp(cap_speed(t, caps_.failsafe.creep_speed_mps));
      } else {
        continue;
      }
    }
    Rollout r;
    if (evaluate(t, in.odom, aim, threshold, near ? &map : nullptr, r)) {
      rollouts.push_back(r);
    }
  }
  std::sort(rollouts.begin(), rollouts.end(),
            [](const Rollout& a, const Rollout& b) { return a.cost < b.cost; });
  proposal.clear_rollouts = static_cast<int>(rollouts.size());

  for (const Rollout& r : rollouts) {
    if (proposal.admission_checks++ >= config_.max_admission_checks) {
      break;
    }
    // Take the candidate as the supervisor would pass it. A slowed version still follows the
    // rollout; one whose translation was refused does not.
    uint32_t flags = 0;
    const Twist out = supervisor.preview(r.velocity, in, flags);
    const bool refused = (flags & (kBlocked | kStaleSensor)) != 0 ||
                         (r.velocity.linear.norm() > 1e-6 && out.linear.norm() <= 1e-6);
    if (!refused) {
      proposal.velocity = out;
      return proposal;
    }
  }
  return proposal;
}

}  // namespace argus::node

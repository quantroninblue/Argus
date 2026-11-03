#include "argus/node/node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

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
      map_(map_config(profile_, caps_, config)),
      tracker_(config.tracker) {
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
    s.view.rows = s.pattern.rows;
    s.view.cols = s.pattern.cols;
    s.view.fov_h_rad = spec.fov_h_deg.value_or(0.0) * M_PI / 180.0;
    s.view.fov_v_rad = spec.kind == capabilities::SensorKind::Range2d ? 0.0 : spec.fov_v_deg.value_or(0.0) * M_PI / 180.0;
    s.view.min_range_m = spec.range_m->min;
    sensors_[spec.id] = std::move(s);
    clock_.max_age[spec.id] = spec.max_age_s;
  }

  // Views are kept out past the stopping reach at top speed by the largest near blind gap a
  // covered direction may have (DeriveOptions::max_near_gap_m), so space seen on the way is
  // still marked as viewed when the body gets there.
  double stop = 0.0;
  const auto& m = caps_.motion;
  for (auto [axis, cap] : {std::pair{capabilities::Axis::X, &m.x}, {capabilities::Axis::Y, &m.y},
                           {capabilities::Axis::Z, &m.z}}) {
    const double top = std::max(std::abs(cap->min_mps), std::abs(cap->max_mps));
    if (cap->enabled() && top > 0.0) {
      stop = std::max(stop, top * (control_period_ + m.latency_s) + m.stopping_distance_m(axis, top));
    }
  }
  view_radius_m_ = std::min(caps_.collision.bounding_radius_m + stop + capabilities::DeriveOptions{}.max_near_gap_m,
                            map_.config().half_width_m);

  body_ = std::make_unique<BodyVolume>(caps_, ground_z_, config_.min_obstacle_height_m,
                                       config_.safety.max_unknown_fraction);
  std::vector<BodyVolume::Sensor> geometry;
  for (const auto& [id, s] : sensors_) {
    geometry.push_back({s.mount, s.view, s.spec->range_m->max});
  }
  body_->set_sensors(std::move(geometry), map_.resolution());
  motion_ = std::make_unique<MotionModel>(caps_.motion);
  if (caps_.motion.space == MotionSpace::Aerial && caps_.max_flight_height_m > 0.0) {
    // The floor is where the drone starts landed, at the odometry origin.
    config_.safety.ceiling_z_m = ground_z_ + caps_.max_flight_height_m;
    config_.planner.ceiling_z_m = config_.safety.ceiling_z_m;
  }
  supervisor_ = std::make_unique<SafetySupervisor>(caps_, *body_, *motion_, config_.safety, control_period_,
                                                   config_.planner.period_s, clock_.max_age);
  planner_ = std::make_unique<LocalPlanner>(caps_, *body_, *motion_, config_.planner);
  if (!config_.plan_in_tick) {
    worker_ = std::thread(&Node::plan_worker, this);
  }
}

Node::~Node() {
  if (worker_.joinable()) {
    {
      std::lock_guard<std::mutex> lock(job_mutex_);
      stop_worker_ = true;
    }
    job_cv_.notify_all();
    worker_.join();
  }
}

void Node::plan_worker() {
  while (true) {
    std::unique_lock<std::mutex> lock(job_mutex_);
    job_cv_.wait(lock, [this] { return stop_worker_ || job_.has_value(); });
    if (stop_worker_) {
      return;
    }
    PlanJob job = std::move(*job_);
    job_.reset();
    job_running_ = true;
    lock.unlock();

    PlanDone out{job.target, Proposal{}, {}};
    {
      std::shared_lock<std::shared_mutex> map_lock(map_mutex_);
      const SafetyInputs in{job.odom, map_, job.clock, job.now, &job.movers, job.odometry_suspect};
      out.proposal = planner_->plan(map_, job.target, job.supervisor, in,
                                    job.following ? &*job.following : nullptr, &job.route);
      out.path = planner_->path();
    }

    lock.lock();
    done_.emplace(std::move(out));
    job_running_ = false;
  }
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
  const bool first = !have_odom_;
  odom_ = odom;
  have_odom_ = true;
  history_.push_back(odom);
  while (!history_.empty() && history_.front().stamp < odom.stamp - config_.odometry_history_s) {
    history_.pop_front();
  }
  {
    // With a planning thread, the map moves only while no plan is reading it; a later
    // message recentres it otherwise.
    std::unique_lock<std::shared_mutex> lock(map_mutex_, std::defer_lock);
    bool may_write = true;
    if (!config_.plan_in_tick) {
      if (first) {
        lock.lock();
      } else {
        may_write = lock.try_lock();
      }
    }
    if (may_write) {
      map_.recenter(odom.pose.position);
      if (first && caps_.assumed_clear_at_start_m > 0.0) {
        double lo = 0.0;
        double hi = 0.0;
        view_band(odom.pose.position, lo, hi);
        mark_zone_viewed(map_, odom.pose.position,
                         caps_.collision.bounding_radius_m + caps_.assumed_clear_at_start_m, lo, hi);
      }
    }
  }
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

bool Node::pose_at(double stamp, Pose4& pose, double* roll, double* pitch) const {
  if (history_.empty() || stamp < history_.front().stamp - 1e-9) {
    return false;
  }
  auto attitude = [&](double r, double p) {
    if (roll) *roll = r;
    if (pitch) *pitch = p;
  };
  const Odometry& last = history_.back();
  if (stamp >= last.stamp) {
    const double dt = stamp - last.stamp;
    if (dt > 0.1) {
      return false;
    }
    pose = MotionModel::integrate(last.pose, last.twist, dt);
    attitude(last.roll, last.pitch);
    return true;
  }
  const auto after = std::lower_bound(history_.begin(), history_.end(), stamp,
                                      [](const Odometry& o, double t) { return o.stamp < t; });
  if (after == history_.begin()) {
    pose = after->pose;
    attitude(after->roll, after->pitch);
    return true;
  }
  const Odometry& a = *(after - 1);
  const Odometry& b = *after;
  const double span = b.stamp - a.stamp;
  const double f = span > 0.0 ? (stamp - a.stamp) / span : 0.0;
  pose.position = a.pose.position + f * (b.pose.position - a.pose.position);
  pose.yaw = wrap_angle(a.pose.yaw + f * wrap_angle(b.pose.yaw - a.pose.yaw));
  attitude(a.roll + f * (b.roll - a.roll), a.pitch + f * (b.pitch - a.pitch));
  return true;
}

void Node::view_band(const Vec3& at, double& lo, double& hi) const {
  // The heights the body can hit: its band for planar bodies, its sphere's extent (plus a
  // cell) for aerial ones, which move vertically too and whose band moves with them.
  if (caps_.motion.space == MotionSpace::Planar) {
    lo = at.z() + body_->z_low_m();
    hi = at.z() + body_->z_high_m();
  } else {
    const double r = body_->radius_m() + map_.resolution();
    lo = at.z() - r;
    hi = at.z() + r;
  }
}

bool Node::on_range_frame(const RangeFrame& frame) {
  if (config_.plan_in_tick) {
    return integrate_frame(frame);
  }
  if (sensors_.find(frame.sensor_id) == sensors_.end()) {
    return false;
  }
  std::unique_lock<std::shared_mutex> lock(map_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    // A plan is reading the map: this frame waits for it. A stuck planner cannot make the
    // queue grow without end (the supervisor's lease has stopped the robot by then).
    deferred_frames_.push_back(frame);
    while (deferred_frames_.size() > 64) {
      deferred_frames_.pop_front();
    }
    return true;
  }
  while (!deferred_frames_.empty()) {
    integrate_frame(deferred_frames_.front());
    deferred_frames_.pop_front();
  }
  return integrate_frame(frame);
}

void Node::flush_deferred_frames() {
  if (deferred_frames_.empty()) {
    return;
  }
  std::unique_lock<std::shared_mutex> lock(map_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;
  }
  while (!deferred_frames_.empty()) {
    integrate_frame(deferred_frames_.front());
    deferred_frames_.pop_front();
  }
}

namespace {

// Time since the previous lap, per named stage, when stage timing is on.
// Frame stages carry the sensor's id ("frame map front_depth"), so each is held to that
// sensor's frame budget.
class Laps {
 public:
  explicit Laps(Node::StageTimes& t, std::string suffix = "") : t_(t.enabled ? &t : nullptr), suffix_(std::move(suffix)) {
    if (t_) last_ = std::chrono::steady_clock::now();
  }
  void operator()(const char* name) {
    if (!t_) return;
    const auto now = std::chrono::steady_clock::now();
    t_->ms[suffix_.empty() ? std::string(name) : std::string(name) + " " + suffix_].push_back(
        std::chrono::duration<double, std::milli>(now - last_).count());
    last_ = now;
  }

 private:
  Node::StageTimes* t_;
  std::string suffix_;
  std::chrono::steady_clock::time_point last_;
};

}  // namespace

void Node::mark_drop_edges(const RangeSensor& s, const Eigen::Isometry3d& to_odom, const Vec3& origin,
                           double floor_z, double ground, const std::vector<size_t>& drop_rays,
                           std::vector<Vec3>& ends, std::vector<bool>& hits, std::vector<bool>& drops) const {
  // A ray that crosses floor level with no floor under it only bounds the edge from beyond:
  // the floor ends somewhere between where the next steeper ray in its column meets the
  // floor and where this one crossed. With sparse rays that stretch is tens of centimetres
  // or more, and the rays passing over it would leave it mapped free. Mark all of it, once
  // per cell; the tracer never clears a cell hit in the same frame. A ray in the bottom row
  // has no steeper neighbour: the stretch is taken as long as the gap to the row above, which
  // is wider than the one below would be.
  const double res = map_.resolution();
  const double drop_h = origin.z() - floor_z;
  const size_t cols = static_cast<size_t>(std::max(0, s.pattern.cols));
  constexpr double kMaxSpanM = 2.0;
  auto crossing = [&](size_t k, Vec2& at) {
    const Vec3 dir = to_odom.linear() * s.pattern.directions[k];
    if (dir.z() >= -1e-6) {
      return false;
    }
    at = (origin + dir * (drop_h / -dir.z())).head<2>();
    return true;
  };
  std::set<std::pair<int, int>> marked;
  for (size_t i : drop_rays) {
    Vec2 far;
    Vec2 near;
    if (cols == 0 || !crossing(i, far)) {
      continue;
    }
    if (i + cols < s.pattern.size()) {
      if (!crossing(i + cols, near)) {
        continue;
      }
    } else {
      Vec2 above;
      if (i < cols || !crossing(i - cols, above)) {
        continue;
      }
      near = far - (above - far);
    }
    const Vec2 span = near - far;
    const double len = std::min(span.norm(), kMaxSpanM);
    if (len < 0.5 * res) {
      continue;
    }
    const Vec2 step = span.normalized() * (0.5 * res);
    Vec2 p = far + step;
    for (double d = 0.5 * res; d < len; d += 0.5 * res, p += step) {
      const Vec3 at(p.x(), p.y(), ground + 0.5 * res);
      const Index3 idx = map_.index_of(at);
      if (!marked.emplace(idx.x(), idx.y()).second) {
        continue;
      }
      ends.push_back(at);
      hits.push_back(true);
      drops.push_back(true);
    }
  }
}

bool Node::frame_healthy(const RangeSensor& s, const RangeFrame& frame, const Pose4& pose, SensorHealth& h) const {
  if (frame.stamp <= h.last_stamp) {
    h.fault = "stamp not moving forward";
    return false;
  }
  size_t considered = 0;
  size_t valid = 0;
  size_t finite = 0;
  for (size_t i = 0; i < frame.ranges.size(); ++i) {
    if (std::isfinite(s.self_hits[i])) {
      continue;
    }
    ++considered;
    const float r = frame.ranges[i];
    valid += std::isnan(r) ? 0 : 1;
    finite += std::isfinite(r) ? 1 : 0;
  }
  h.last_stamp = frame.stamp;
  if (considered > 0 && static_cast<double>(valid) < (1.0 - config_.max_invalid_share) * considered) {
    h.fault = "nearly every ray invalid";
    return false;
  }
  // Compared byte for byte: a frozen driver resends the same buffer, NaN rays included.
  const bool same = finite >= 10 && h.last_ranges.size() == frame.ranges.size() &&
                    std::memcmp(h.last_ranges.data(), frame.ranges.data(), frame.ranges.size() * sizeof(float)) == 0;
  if (!same) {
    h.repeating = false;
    h.last_ranges = frame.ranges;
  } else if (!h.repeating) {
    h.repeating = true;
    h.repeat_from = pose;
  } else if ((pose.position - h.repeat_from.position).norm() > config_.frozen_motion_m ||
             std::abs(wrap_angle(pose.yaw - h.repeat_from.yaw)) > config_.frozen_turn_rad) {
    h.fault = "frames repeat while the robot moves";
    return false;
  }
  h.fault.clear();
  return true;
}

void Node::cross_check_odometry(const std::string& sensor_id, double stamp, const Pose4& pose, const std::vector<Vec3>& ends,
                                const std::vector<bool>& hits, const std::vector<bool>& drops) {
  // Static returns of this frame (movers were taken out above, drops are not surfaces), in a
  // band around the body's own height: walls and pillars there pin down position and
  // heading; a floor or ceiling seen from a drone says nothing about either and only adds
  // chance matches.
  std::vector<Vec3> points;
  const double band_lo = pose.position.z() - 0.3;
  const double band_hi = pose.position.z() + 1.0;
  for (size_t k = 0; k < ends.size(); ++k) {
    if (hits[k] && !drops[k] && ends[k].z() >= band_lo && ends[k].z() <= band_hi) {
      points.push_back(ends[k]);
    }
  }
  OdometryKeyframe& key = odom_keys_[sensor_id];
  const bool have_key = !key.points.empty();
  const double travel = (pose.position - key.pose.position).head<2>().norm();
  const double turn = std::abs(wrap_angle(pose.yaw - key.pose.yaw));
  if (have_key && travel < config_.odometry_check_travel_m && turn < config_.odometry_check_turn_rad &&
      stamp - key.stamp < config_.odometry_check_period_s) {
    return;
  }
  if (have_key && points.size() >= 40) {
    // The keyframe's returns on a 5 cm grid, widened by one cell so two scans of the same
    // wall from different places still meet.
    constexpr double kCell = 0.05;
    // Coarse in height: a body that climbs or dips meets the same wall at other heights.
    constexpr double kLayer = 0.5;
    auto key_of = [](int x, int y, int z) {
      return (static_cast<int64_t>(x) & 0x1FFFFF) | ((static_cast<int64_t>(y) & 0x1FFFFF) << 21) |
             ((static_cast<int64_t>(z) & 0x1FFFFF) << 42);
    };
    std::unordered_set<int64_t> grid;
    const size_t key_stride = std::max<size_t>(1, key.points.size() / 600);
    for (size_t k = 0; k < key.points.size(); k += key_stride) {
      const Vec3& p = key.points[k];
      const int x = static_cast<int>(std::floor(p.x() / kCell));
      const int y = static_cast<int>(std::floor(p.y() / kCell));
      const int z = static_cast<int>(std::floor(p.z() / kLayer));
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          grid.insert(key_of(x + dx, y + dy, z));
        }
      }
    }
    // At most 200 of this frame's returns, spread over the frame.
    const size_t stride = std::max<size_t>(1, points.size() / 200);
    std::vector<Vec2> rel;
    std::vector<int> layer;
    for (size_t k = 0; k < points.size(); k += stride) {
      rel.push_back(points[k].head<2>() - pose.position.head<2>());
      layer.push_back(static_cast<int>(std::floor(points[k].z() / kLayer)));
    }
    auto score = [&](double dx, double dy, double dyaw) {
      const double c = std::cos(dyaw);
      const double sn = std::sin(dyaw);
      int n = 0;
      for (size_t k = 0; k < rel.size(); ++k) {
        const Vec2 q(c * rel[k].x() - sn * rel[k].y() + pose.position.x() + dx,
                     sn * rel[k].x() + c * rel[k].y() + pose.position.y() + dy);
        n += grid.count(key_of(static_cast<int>(std::floor(q.x() / kCell)),
                               static_cast<int>(std::floor(q.y() / kCell)), layer[k]));
      }
      return n;
    };
    const int zero = score(0.0, 0.0, 0.0);
    int best = zero;
    double best_dx = 0.0;
    double best_dy = 0.0;
    double best_yaw = 0.0;
    constexpr double kDeg = M_PI / 180.0;
    // Coarse to fine: the turn alone over a wide range, then the shift around the best turn.
    // Ties go to the smaller correction.
    int coarse = 0;
    for (int a = -8; a <= 8; ++a) {
      const int n = score(0.0, 0.0, a * kDeg);
      if (n > best || (n == best && std::abs(a) < std::abs(coarse))) {
        best = n;
        coarse = a;
        best_yaw = a * kDeg;
      }
    }
    for (int a = coarse - 1; a <= coarse + 1; ++a) {
      for (int i = -5; i <= 5; ++i) {
        for (int j = -5; j <= 5; ++j) {
          const int n = score(i * kCell, j * kCell, a * kDeg);
          const double size = std::hypot(i, j) + std::abs(a);
          if (n > best || (n == best && size < std::hypot(best_dx, best_dy) / kCell + std::abs(best_yaw) / kDeg)) {
            best = n;
            best_dx = i * kCell;
            best_dy = j * kCell;
            best_yaw = a * kDeg;
          }
        }
      }
    }
    // A verdict needs enough of the frame to land on what the keyframe saw, and a clear best
    // match: along a bare corridor every shift along it scores the same.
    ++odom_check_.checks;
    odom_check_.last_zero = zero;
    odom_check_.last_best = best;
    odom_check_.last_points = static_cast<int>(rel.size());
    const bool conclusive = best >= 30 && best >= static_cast<int>(0.3 * rel.size());
    if (conclusive) {
      const double shift = std::hypot(best_dx, best_dy);
      const bool off = best >= zero + std::max(8, zero / 6) &&
                       (shift > config_.odometry_max_shift_m || std::abs(best_yaw) > config_.odometry_max_turn_rad);
      ++odom_check_.verdicts;
      odom_check_.shift_m = shift;
      odom_check_.turn_rad = best_yaw;
      if (off) {
        odom_agreements_ = 0;
        if (++odom_strikes_ >= config_.odometry_strikes) {
          odom_check_.suspect = true;
        }
      } else {
        odom_strikes_ = 0;
        if (++odom_agreements_ >= config_.odometry_strikes) {
          odom_check_.suspect = false;
        }
      }
    }
  }
  key.pose = pose;
  key.stamp = stamp;
  key.points = std::move(points);
}

std::map<std::string, std::string> Node::sensor_faults() const {
  std::map<std::string, std::string> out;
  for (const auto& [id, h] : health_) {
    if (!h.fault.empty()) {
      out[id] = h.fault;
    }
  }
  return out;
}

bool Node::integrate_frame(const RangeFrame& frame) {
  const auto it = sensors_.find(frame.sensor_id);
  if (it == sensors_.end()) {
    return false;
  }
  const RangeSensor& s = it->second;
  if (frame.ranges.size() != s.pattern.size()) {
    return false;
  }
  Laps lap(stage_times_, frame.sensor_id);
  SensorHealth& health = health_[frame.sensor_id];
  Pose4 pose;
  double roll = 0.0;
  double pitch = 0.0;
  if (!pose_at(frame.stamp, pose, &roll, &pitch)) {
    if (!history_.empty() && frame.stamp > history_.back().stamp) {
      health.fault = "stamp ahead of odometry";
    }
    return false;
  }
  const bool healthy = frame_healthy(s, frame, pose, health);
  lap("frame health");
  if (!healthy) {
    return false;
  }
  // The body as it really sits, tilted as odometry says, so returns land where they are.
  Eigen::Isometry3d body = pose.transform();
  body.linear() = (Eigen::AngleAxisd(pose.yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
                   Eigen::AngleAxisd(roll, Vec3::UnitX()))
                      .toRotationMatrix();
  const Eigen::Isometry3d to_odom = body * s.mount;
  const Vec3 origin = to_odom.translation();
  const double min_range = s.spec->range_m->min;
  const double max_range = s.spec->range_m->max;
  const bool planar = caps_.motion.space == MotionSpace::Planar;
  const double ground = pose.position.z() + ground_z_ + config_.min_obstacle_height_m;

  std::vector<Vec3> ends;
  std::vector<bool> hits;
  std::vector<bool> drops;  // hits placed on a floor edge, never taken for something moving
  std::vector<size_t> drop_rays;
  std::vector<Index3> floor_cells;  // returns from the floor: it is there
  ends.reserve(frame.ranges.size());
  hits.reserve(frame.ranges.size());
  drops.reserve(frame.ranges.size());
  const double floor_z = pose.position.z() + ground_z_;
  // How far each ray saw free space, for telling cells between rays from unseen ones.
  std::vector<float> free_m(frame.ranges.size(), std::numeric_limits<float>::quiet_NaN());
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
    if (!hit && !s.spec->no_return_is_free.value_or(config_.no_return_is_free)) {
      continue;
    }
    free_m[i] = static_cast<float>(hit ? r : max_range);
    Vec3 end = origin + dir * (hit ? r : max_range);
    bool drop = false;
    if (planar && hit && config_.drop_depth_m > 0.0 && dir.z() < -1e-6 && origin.z() > floor_z &&
        end.z() < floor_z - config_.drop_depth_m) {
      // The floor was not there where this ray crossed it: an edge at or before that point.
      // Marked just above the ground cut, in the layers the body is checked against. Rays with
      // no return are left alone: dark or shiny floors often give none, and a drop deeper
      // than the sensor's range goes unseen.
      const double t = (floor_z - origin.z()) / dir.z();
      const Vec3 cross = origin + dir * t;
      end = Vec3(cross.x(), cross.y(), ground + 0.5 * map_.resolution());
      free_m[i] = static_cast<float>(std::max(0.0, t - map_.resolution()));
      hit = true;
      drop = true;
      drop_rays.push_back(i);
    } else if (hit && planar && end.z() < ground) {
      hit = false;
      floor_cells.push_back(map_.index_of(end));
    }
    ends.push_back(end);
    hits.push_back(hit);
    drops.push_back(drop);
  }
  lap("frame rays");
  if (!drop_rays.empty()) {
    mark_drop_edges(s, to_odom, origin, floor_z, ground, drop_rays, ends, hits, drops);
  }
  lap("frame drop edges");

  // Returns landing in ever-free space are something moving through it (Dynablox): the
  // tracker follows them, and they stay out of the map (the ray still clears the space up to
  // them). A cell that keeps taking such returns for static_after_s is something that has
  // stopped there for good: it loses ever-free and from then on is mapped like any obstacle.
  if (tracking()) {
    std::vector<Vec3> dynamic_returns;
    // One return per floor cell and half metre of height: enough to follow walkers and know
    // how tall they are, and keeps a depth image cheap.
    std::map<std::tuple<int, int, int>, bool> seen_column;
    const int height_step = std::max(1, static_cast<int>(std::lround(0.5 / map_.resolution())));
    const double cell = map_.resolution();
    // The layer holding the ground cut is part floor, which ground returns mark free, and part
    // the foot of whatever stands there: a wall's base would pass for something moving. A
    // return there counts as moving only when the cell above it is ever-free too, which a
    // wall's base never has and a low cart or robot passing under the open space does.
    const int ground_layer = map_.index_of(Vec3(origin.x(), origin.y(), ground)).z();
    for (size_t k = 0; k < ends.size(); ++k) {
      if (!hits[k] || drops[k]) {
        continue;
      }
      const Index3 idx = map_.index_of(ends[k]);
      if ((planar && idx.z() < ground_layer) || !map_.ever_free(idx)) {
        continue;
      }
      if (planar && idx.z() == ground_layer && !map_.ever_free(Index3(idx.x(), idx.y(), idx.z() + 1))) {
        continue;
      }
      DynamicCell& d = dynamic_cells_.try_emplace({idx.x(), idx.y(), idx.z()}, DynamicCell{frame.stamp, frame.stamp})
                           .first->second;
      d.last = frame.stamp;
      if (frame.stamp - d.since >= config_.static_after_s) {
        map_.revoke_ever_free(idx);
        dynamic_cells_.erase({idx.x(), idx.y(), idx.z()});
        continue;  // integrated as a hit: it is part of the map now
      }
      const int band = idx.z() >= 0 ? idx.z() / height_step : (idx.z() - height_step + 1) / height_step;
      if (seen_column.emplace(std::make_tuple(idx.x(), idx.y(), band), true).second) {
        dynamic_returns.push_back(ends[k]);
      }
      const Vec3 d_ray = ends[k] - origin;
      const double len = d_ray.norm();
      ends[k] = len > cell ? origin + d_ray * ((len - cell) / len) : origin;
      hits[k] = false;
    }
    // A cell that went a few frames without a dynamic return has been left: start afresh.
    for (auto it = dynamic_cells_.begin(); it != dynamic_cells_.end();) {
      it = frame.stamp - it->second.last > 0.3 ? dynamic_cells_.erase(it) : std::next(it);
    }
    tracker_.update(frame.stamp, dynamic_returns);
  }
  if (tracking()) {
    const Pose4 at = pose;
    map_.set_vertically_unseen([this, at](const Index3& n) {
      bool covered = false;
      bool in_view = false;
      return body_->out_of_vertical_view(at, map_.center_of(n), covered, in_view);
    });
  }
  lap("frame tracking");
  if (config_.check_odometry) {
    cross_check_odometry(frame.sensor_id, frame.stamp, pose, ends, hits, drops);
  }
  lap("frame odometry alarm");
  map_.integrate(origin, ends, hits);
  map_.set_vertically_unseen(nullptr);
  for (const Index3& c : floor_cells) {
    map_.clear_drop_column(c);
  }
  for (size_t k = 0; k < ends.size(); ++k) {
    if (drops[k]) {
      map_.mark_drop(map_.index_of(ends[k]));
    }
  }
  if (tracking()) {
    forget_lost(frame.stamp);
  }
  lap("frame map");
  RangeView view;
  view.layout = &s.view;
  view.odom_to_sensor = to_odom.inverse();
  view.free_m = &free_m;
  view.cell_m = map_.resolution();
  double lo = 0.0;
  double hi = 0.0;
  view_band(pose.position, lo, hi);
  mark_viewed(map_, view, pose.position, view_radius_m_, lo, hi);
  lap("frame view");
  double& last = clock_.last_frame[frame.sensor_id];
  last = std::max(last, frame.stamp);
  return true;
}

bool Node::tracking() const { return config_.track_movers; }

bool Node::reaches_height(const TrackedObstacle& o, double z, double margin) const {
  // A planar body shares the floor with whatever walks there. An aerial one is affected only
  // when its sphere comes within `margin` of the heights the thing was seen at.
  if (caps_.motion.space == MotionSpace::Planar) {
    return true;
  }
  const double r = body_->radius_m();
  return z + r + margin >= o.z_low_m && z - r - margin <= o.z_high_m;
}

void Node::forget_lost(double stamp) {
  // A walker lost from view may be standing where it was heading: that space goes back to
  // never looked at, so the robot does not drive into it until a sensor has looked again.
  const double cell = map_.resolution();
  for (const TrackedObstacle& m : tracker_.take_lost()) {
    double lo = 0.0;
    double hi = 0.0;
    if (caps_.motion.space == MotionSpace::Planar) {
      view_band(odom_.pose.position, lo, hi);
    } else {
      lo = m.z_low_m - cell;
      hi = m.z_high_m + cell;
    }
    const double r = m.radius_m + tracker_.config().cluster_gap_m;
    for (double x = -r; x <= r + 1e-9; x += cell) {
      for (double y = -r; y <= r + 1e-9; y += cell) {
        if (x * x + y * y > r * r) {
          continue;
        }
        for (double z = lo; z <= hi + 1e-9; z += cell) {
          map_.forget(map_.index_of(Vec3(m.position.x() + x, m.position.y() + y, z)));
        }
      }
    }
    last_moving_.erase(m.id);
    yielded_for_.erase(m.id);
  }
  (void)stamp;
}

bool Node::update_yield(double now) {
  // Movers for the supervisor: everything moving, and anything that moved lately, held a bit
  // wider since it may set off again. A body that has just stopped on the way ahead makes
  // Argo hold still for yield_wait_s first.
  movers_.clear();
  if (!tracking()) {
    return false;
  }
  // Near the planned path within the lookahead.
  const auto on_way_ahead = [&](const TrackedObstacle& o) {
    const double reach = body_->radius_m() + o.radius_m + config_.restart_margin_m;
    double along = 0.0;
    Vec3 prev = odom_.pose.position;
    for (const Vec3& p : path()) {
      along += (p - prev).head<2>().norm();
      prev = p;
      if (along > config_.yield_lookahead_m) {
        return false;
      }
      if ((p.head<2>() - o.position).norm() < reach && reaches_height(o, p.z(), config_.restart_margin_m)) {
        return true;
      }
    }
    return false;
  };
  // The yield lasts its time unless the body it is for is lost or leaves the way ahead. A
  // standing person seen from a new side drifts and may read as moving for a moment; that
  // alone does not end it.
  bool yield_target_seen = false;
  while (!yielded_at_.empty() && now - yielded_at_.front().second > config_.recently_moved_s) {
    yielded_at_.erase(yielded_at_.begin());
  }
  for (TrackedObstacle o : tracker_.all(now)) {
    if (o.id == yield_id_ && now < yield_until_ && on_way_ahead(o)) {
      yield_target_seen = true;
    }
    if (o.moving) {
      last_moving_[o.id] = now;
      o.has_moved = true;
      movers_.push_back(o);
      continue;
    }
    // Not moving (or not yet confirmed): its returns are kept out of the map, so the
    // supervisor must see it here, standing where it was last seen. The filter's prediction
    // would carry it on along its last walking velocity while its returns go into the map.
    o.velocity.setZero();
    if (!o.trail.empty()) {
      o.position = o.trail.back();
    }
    const auto lm = last_moving_.find(o.id);
    if (lm == last_moving_.end() || now - lm->second > config_.recently_moved_s) {
      movers_.push_back(o);
      continue;
    }
    // Waited here already: the same body, picked up again as a new track (seen from a new
    // side, or lost for a moment), is not waited for twice.
    bool waited_here = false;
    for (const auto& [where, when] : yielded_at_) {
      waited_here = waited_here || (now - when < config_.recently_moved_s &&
                                    (where - o.position).norm() < o.radius_m + config_.restart_margin_m + 0.5);
    }
    if (!yielded_for_[o.id] && !waited_here && now - lm->second < 1.0 && on_way_ahead(o)) {
      yielded_for_[o.id] = true;
      yielded_at_.emplace_back(o.position, now);
      yield_id_ = o.id;
      yield_until_ = now + config_.yield_wait_s;
      pause_pending_ = true;
      pause_at_ = Vec3(o.position.x(), o.position.y(), odom_.pose.position.z());
      yield_target_seen = true;
    }
    o.radius_m += config_.restart_margin_m;
    o.has_moved = true;
    movers_.push_back(o);
  }
  // People who stopped and stood long enough to be mapped lose their track, not their being
  // people: their last spot is kept (standing, wider) for recently_moved_s, so the robot still
  // slows past and keeps clear of them. A track lost while walking leaves nothing behind.
  std::map<int, bool> present;
  for (const TrackedObstacle& o : movers_) {
    present[o.id] = true;
    if (o.has_moved) {
      people_[o.id] = {o, now};
    }
  }
  for (auto it = people_.begin(); it != people_.end();) {
    const auto& [last, seen] = it->second;
    if (now - seen > config_.recently_moved_s) {
      it = people_.erase(it);
      continue;
    }
    if (!present.count(it->first) && last.velocity.norm() <= 1e-3) {
      TrackedObstacle ghost = last;
      ghost.remembered = true;
      movers_.push_back(ghost);
    }
    ++it;
  }
  if (!yield_target_seen) {
    yield_until_ = -1e9;  // it moved on or was lost: no reason to wait
  }
  return now < yield_until_;
}

bool Node::set_plan(const Plan& plan, double now) {
  std::string reason;
  if (!caps_.safety_ready) {
    reason = !caps_.missing_fields.empty() ? "profile is missing " + caps_.missing_fields.front()
                                           : "profile has unreviewed LLM draft fields";
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

  if (pause_pending_) {
    std::ostringstream where;
    where.setf(std::ios::fixed);
    where.precision(2);
    where << "waiting for something that stopped on the way at " << pause_at_.x() << " " << pause_at_.y();
    events.push_back({now, EventKind::Paused, target->id, FailureCause::TemporaryObstruction, where.str()});
    pause_pending_ = false;
  }

  // No clear motion toward the waypoint, or the waypoint itself blocked, while not waiting for
  // a person and with nothing else wrong: stop trying and tell the hub once, so it routes the
  // robot another way.
  const bool stuck = proposal_.has_target && proposal_.velocity.is_zero() && !(flags & kYielding) &&
                     hard_cause(flags) == FailureCause::None &&
                     (proposal_.target_blocked || proposal_.off_route ||
                      (proposal_.candidates > 0 && proposal_.clear_rollouts == 0));
  if (!stuck) {
    stuck_since_ = -1.0;
  } else if (stuck_since_ < 0.0) {
    stuck_since_ = now;
  } else if (now - stuck_since_ >= config_.stuck_report_s && stuck_reported_ != key) {
    events.push_back({now, EventKind::Blocked, target->id, FailureCause::TemporaryObstruction,
                      proposal_.target_blocked ? "the waypoint is blocked"
                      : proposal_.off_route    ? "no way around near the hub's path"
                                               : "no clear motion along the path"});
    stuck_reported_ = key;
    last_progress_ = now;
  }
}

std::vector<Vec3> Node::route_for(const Waypoint& target, const Waypoint* following) {
  const std::string key = std::to_string(mission_.generation()) + "/" + target.id;
  if (key != leg_key_) {
    leg_key_ = key;
    leg_start_ = odom_.pose.position;
  }
  std::vector<Vec3> route{leg_start_, target.position};
  if (following != nullptr) {
    route.push_back(following->position);
  }
  return route;
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

bool Node::clear_line(const Vec3& a, const Vec3& b) const {
  Vec3 d = b - a;
  const bool planar = caps_.motion.space == MotionSpace::Planar;
  if (planar) {
    d.z() = 0.0;
  }
  const double len = d.norm();
  const double yaw = len > 1e-6 ? std::atan2(d.y(), d.x()) : odom_.pose.yaw;
  const int steps = std::max(1, static_cast<int>(std::ceil(len / map_.resolution())));
  for (int k = 0; k <= steps; ++k) {
    Pose4 p;
    p.position = a + (b - a) * (static_cast<double>(k) / steps);
    if (planar) {
      p.position.z() = odom_.pose.position.z();
    }
    p.yaw = yaw;
    if (body_->contact(map_, p, &odom_.pose) != Contact::Free) {
      return false;
    }
  }
  return true;
}

std::vector<Vec3> Node::shortcut(const std::vector<Vec3>& trail) const {
  // String pulling: from each kept point, go straight to the farthest of the next few points
  // the padded body can reach in a line through space it has seen free. Where the local map
  // does not know the space (outside it, or never seen), the trail is kept as walked.
  constexpr size_t kLookahead = 20;
  if (trail.size() < 3) {
    return trail;
  }
  std::vector<Vec3> out{trail.front()};
  size_t i = 0;
  while (i + 1 < trail.size()) {
    size_t next = i + 1;
    for (size_t j = std::min(trail.size() - 1, i + kLookahead); j > i + 1; --j) {
      if (clear_line(trail[i], trail[j])) {
        next = j;
        break;
      }
    }
    out.push_back(trail[next]);
    i = next;
  }
  return out;
}

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
  back = shortcut(back);
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
  planned_in_tick_ = false;
  if (!config_.plan_in_tick) {
    flush_deferred_frames();
  }
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
  Laps lap(stage_times_);
  const bool yielding = update_yield(now);
  lap("tick movers");
  const SafetyInputs in{odom_, map_, clock_, now, &movers_, odom_check_.suspect};

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
    if (!config_.plan_in_tick) {
      std::lock_guard<std::mutex> lock(job_mutex_);
      done_.reset();
    }
  } else if (config_.plan_in_tick) {
    if (now - last_plan_time_ >= config_.planner.period_s - 1e-9 || !proposal_.has_target) {
      const Waypoint* following = target == mission_.target() ? mission_.after_target() : nullptr;
      const std::vector<Vec3> route = route_for(*target, following);
      proposal_ = planner_->plan(map_, *target, *supervisor_, in, following, &route);
      path_ = planner_->path();
      last_plan_time_ = now;
      planned_in_tick_ = true;
    }
  } else {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (done_) {
      // A plan for another goal (replaced, cancelled, or the creep retrace begun or ended
      // meanwhile) is dropped.
      if (done_->target.id == target->id && (done_->target.position - target->position).norm() < 1e-9) {
        proposal_ = std::move(done_->proposal);
        path_ = std::move(done_->path);
      }
      done_.reset();
    }
    const bool due = now - last_plan_time_ >= config_.planner.period_s - 1e-9 || !proposal_.has_target;
    if (due && !job_running_ && !job_) {
      const Waypoint* following = target == mission_.target() ? mission_.after_target() : nullptr;
      job_.emplace(PlanJob{*target, following ? std::optional<Waypoint>(*following) : std::nullopt, odom_, clock_,
                           movers_, now, *supervisor_, odom_check_.suspect, route_for(*target, following)});
      last_plan_time_ = now;
      job_cv_.notify_one();
    }
  }

  if (yielding && target) {
    proposal_.velocity = Twist{};
  }
  lap("tick plan");
  SafetySupervisor::Verdict verdict = supervisor_->check(proposal_.velocity, proposal_.inputs_stamp, in);
  lap("tick supervisor");
  if (yielding && target) {
    verdict.flags |= kYielding;
  }
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
      std::string detail;
      if (raised & kStaleSensor) {
        for (const auto& [id, why] : sensor_faults()) {
          detail += (detail.empty() ? "" : "; ") + id + ": " + why;
        }
      }
      step_events.push_back({now, EventKind::Blocked, target->id, hard_cause(raised), detail});
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

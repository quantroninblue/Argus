#include "argus/node/safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus::node {

using capabilities::CoverageBin;
using capabilities::kNever;
using capabilities::MotionSpace;

namespace {

constexpr double kMoving = 1e-6;
// Directions seen so briefly that the body could not usefully move are treated as uncovered.
constexpr double kMinSightSpeed = 0.02;

const CoverageBin* find_bin(const capabilities::SensingCapability& sensing, MotionSpace space,
                            const Vec3& dir) {
  const auto& bins = sensing.bins;
  if (bins.empty()) {
    return nullptr;
  }
  double az = std::atan2(dir.y(), dir.x()) * 180.0 / M_PI;
  if (az < 0.0) {
    az += 360.0;
  }
  if (space == MotionSpace::Planar) {
    const int n = static_cast<int>(bins.size());
    const int i = static_cast<int>(std::lround(az / (360.0 / n))) % n;
    return &bins[i];
  }
  size_t per_row = 0;
  while (per_row < bins.size() && bins[per_row].elevation_deg == bins[0].elevation_deg) {
    ++per_row;
  }
  const size_t rows = bins.size() / per_row;
  const double el = std::atan2(dir.z(), dir.head<2>().norm()) * 180.0 / M_PI;
  const double el_step = rows > 1 ? (bins.back().elevation_deg - bins[0].elevation_deg) / (rows - 1) : 1.0;
  const size_t row = std::min(rows - 1, static_cast<size_t>(std::max(
                                            0L, std::lround((el - bins[0].elevation_deg) / el_step))));
  const size_t col = static_cast<size_t>(std::lround(az / (360.0 / per_row))) % per_row;
  return &bins[row * per_row + col];
}

// Largest share of the change from `last` to `v` this axis allows in one period. Slowing down
// is always allowed; speeding up is limited to the body's acceleration.
double allowed_share(double v, double last, double accel, double dt) {
  const double delta = std::abs(v - last);
  if (delta < 1e-12 || (last * v >= 0.0 && std::abs(v) <= std::abs(last))) {
    return 1.0;
  }
  return accel > 0.0 ? std::min(1.0, accel * dt / delta) : 0.0;
}

}  // namespace

std::vector<std::string> flag_names(uint32_t flags) {
  static const std::pair<SafetyFlag, const char*> names[] = {
      {kNoTarget, "no_target"},       {kDeadline, "deadline"},   {kLocalization, "localization"},
      {kNotSafetyReady, "not_safety_ready"}, {kClamped, "clamped"}, {kUncovered, "uncovered"},
      {kStaleSensor, "stale_sensor"}, {kCreep, "creep"},         {kSlowed, "slowed"},
      {kBlocked, "blocked"},     {kShortSight, "short_sight"},
  };
  std::vector<std::string> out;
  for (const auto& [flag, name] : names) {
    if (flags & flag) {
      out.emplace_back(name);
    }
  }
  return out;
}

bool SensorClock::fresh(const std::string& id, double now) const {
  const auto last = last_frame.find(id);
  const auto age = max_age.find(id);
  return last != last_frame.end() && age != max_age.end() && now - last->second <= age->second;
}

SafetySupervisor::SafetySupervisor(const capabilities::Capabilities& caps, const BodyVolume& body,
                                   const MotionModel& motion, const SafetyConfig& config,
                                   double control_period_s, double planner_period_s,
                                   const std::map<std::string, double>& sensor_max_age)
    : caps_(caps), body_(body), motion_(motion), config_(config), period_(control_period_s) {
  sensor_max_age_ = sensor_max_age;
  bool sees = false;
  for (const auto& b : caps.sensing.bins) {
    sees = sees || b.full_height_reach_m > 0.0;
  }
  creep_allowed_ = sees && caps.failsafe.uncovered_motion == capabilities::UncoveredMotion::Creep &&
                   caps.failsafe.creep_speed_mps > 0.0;
  creep_budget_ = std::min(config_.creep_distance_m, 2.0 * body_.body_radius_m());
  creep_left_ = creep_budget_;
  lease_ = config_.command_lease_s;
  if (lease_ <= 0.0) {
    const double timeout = caps.control.controller_timeout_s > 0.0 ? caps.control.controller_timeout_s : 0.3;
    lease_ = std::max(timeout, 2.5 * planner_period_s);
  }
}

Coverage SafetySupervisor::coverage(const Vec3& direction, const SafetyInputs& in) const {
  const CoverageBin* bin = find_bin(caps_.sensing, caps_.motion.space, direction);
  if (!bin || bin->full_height_reach_m <= 0.0 || sight_speed_limit(direction) < kMinSightSpeed) {
    return Coverage::Uncovered;
  }
  for (const std::string& id : bin->sensors) {
    if (in.sensors.fresh(id, in.now)) {
      return Coverage::Covered;
    }
  }
  return Coverage::Stale;
}

double SafetySupervisor::sight_speed_limit(const Vec3& direction) const {
  const CoverageBin* bin = find_bin(caps_.sensing, caps_.motion.space, direction);
  if (!bin || bin->full_height_reach_m <= 0.0) {
    return 0.0;
  }
  double frame_age = 0.0;
  for (const std::string& id : bin->sensors) {
    const auto age = sensor_max_age_.find(id);
    frame_age = std::max(frame_age, age == sensor_max_age_.end() ? 0.0 : age->second);
  }
  // Braking along this direction is limited by the weakest axis that carries part of it.
  const auto& m = caps_.motion;
  const capabilities::AxisCapability* axes[] = {&m.x, &m.y, &m.z};
  double decel = std::numeric_limits<double>::infinity();
  for (int a = 0; a < 3; ++a) {
    if (std::abs(direction[a]) > 0.1) {
      decel = std::min(decel, axes[a]->decel_mps2);
    }
  }
  if (!(decel > 0.0) || !std::isfinite(decel)) {
    return 0.0;
  }
  // Largest v with v * delay + v^2 / (2 * decel) <= reach.
  const double delay = frame_age + period_ + m.latency_s;
  const double reach = bin->full_height_reach_m;
  return decel * (-delay + std::sqrt(delay * delay + 2.0 * reach / decel));
}

bool SafetySupervisor::envelope_clear(const Twist& v, const SafetyInputs& in, bool occupied_only) const {
  const Pose4& here = in.odom.pose;
  const double coast = period_ + caps_.motion.latency_s;
  const auto path = motion_.stopping_path(here, v, coast, config_.envelope_spacing_m, body_.radius_m());
  for (size_t i = 1; i < path.size(); ++i) {
    const Contact c = body_.contact(in.map, path[i], &here);
    if (c == Contact::Occupied || (c == Contact::Unknown && !occupied_only)) {
      return false;
    }
  }
  return true;
}

Twist SafetySupervisor::limit_acceleration(const Twist& v) const {
  // One share for every axis, so the direction of travel stays the one that was checked.
  const auto& m = caps_.motion;
  const double share = std::min({allowed_share(v.linear.x(), last_.linear.x(), m.x.accel_mps2, period_),
                                 allowed_share(v.linear.y(), last_.linear.y(), m.y.accel_mps2, period_),
                                 allowed_share(v.linear.z(), last_.linear.z(), m.z.accel_mps2, period_),
                                 allowed_share(v.yaw_rate, last_.yaw_rate, m.yaw_accel_radps2, period_)});
  Twist out;
  out.linear = last_.linear + share * (v.linear - last_.linear);
  out.yaw_rate = last_.yaw_rate + share * (v.yaw_rate - last_.yaw_rate);
  return motion_.clamp(out);
}

Twist SafetySupervisor::gate(const Twist& in_v, const SafetyInputs& in, uint32_t& flags) const {
  Twist v = in_v;
  bool occupied_only = false;
  const double speed = v.linear.norm();
  if (speed > kMoving) {
    const Vec3 dir = v.linear / speed;
    switch (coverage(dir, in)) {
      case Coverage::Covered: {
        const double limit = sight_speed_limit(dir);
        if (speed > limit) {
          flags |= kShortSight;
          v.linear *= limit / speed;
        }
        break;
      }
      case Coverage::Stale:
        flags |= kStaleSensor;
        v.linear.setZero();
        break;
      case Coverage::Uncovered:
        flags |= kUncovered;
        if (creep_available()) {
          flags |= kCreep;
          occupied_only = true;
          if (speed > caps_.failsafe.creep_speed_mps) {
            v.linear *= caps_.failsafe.creep_speed_mps / speed;
          }
        } else {
          v.linear.setZero();
        }
        break;
    }
    v = motion_.clamp(v);
  }

  if (v.linear.norm() > kMoving) {
    for (double scale : {1.0, 0.5, 0.25}) {
      Twist scaled = v;
      scaled.linear *= scale;
      scaled = motion_.clamp(scaled);
      if (envelope_clear(scaled, in, occupied_only)) {
        if (scale < 1.0) {
          flags |= kSlowed;
        }
        return scaled;
      }
    }
    flags |= kBlocked;
    v.linear.setZero();
    v = motion_.clamp(v);
  }

  // Turning in place: the corners sweep space too, so the same rule as for moving holds: no
  // known obstacle, and no space no sensor has looked at (creep keeps its occupied-only check).
  if (v.yaw_rate != 0.0 && body_.planar() && !envelope_clear(v, in, occupied_only)) {
    flags |= kBlocked;
    v.yaw_rate = 0.0;
  }
  return v;
}

void SafetySupervisor::track_creep(const Vec3& position) {
  // Distance moved since the last check is charged to the creep stretch if that command was a
  // creep; any move in a covered direction starts a fresh stretch.
  if (have_last_position_) {
    const double moved = (position - last_position_).norm();
    if (last_creep_) {
      creep_left_ = std::max(0.0, creep_left_ - moved);
    } else if (last_.linear.norm() > kMoving) {
      creep_left_ = creep_budget_;
    }
  }
  have_last_position_ = true;
  last_position_ = position;
}

Twist SafetySupervisor::preview(const Twist& v, const SafetyInputs& in, uint32_t& flags) const {
  return gate(motion_.clamp(v), in, flags);
}

bool SafetySupervisor::admits(const Twist& v, const SafetyInputs& in) const {
  uint32_t flags = 0;
  const Twist out = preview(v, in, flags);
  return (flags & (kBlocked | kStaleSensor | kSlowed | kShortSight)) == 0 &&
         ((flags & kUncovered) == 0 || (flags & kCreep) != 0) &&
         (out.linear - v.linear).norm() < 1e-6 && std::abs(out.yaw_rate - v.yaw_rate) < 1e-6;
}

SafetySupervisor::Verdict SafetySupervisor::check(const Twist& proposed, double inputs_stamp,
                                                  const SafetyInputs& in) {
  Verdict verdict;
  if (!caps_.safety_ready) {
    verdict.flags |= kNotSafetyReady;
  }
  if (in.now - inputs_stamp > lease_) {
    verdict.flags |= kDeadline;
  }
  if (!in.odom.healthy || in.now - in.odom.stamp > config_.odometry_max_age_s) {
    verdict.flags |= kLocalization;
  }
  track_creep(in.odom.pose.position);
  if (verdict.flags) {
    last_ = Twist{};
    last_creep_ = false;
    return verdict;
  }

  Twist v = motion_.clamp(proposed);
  const Twist limited = limit_acceleration(v);
  if ((limited.linear - proposed.linear).norm() > 1e-9 ||
      std::abs(limited.yaw_rate - proposed.yaw_rate) > 1e-9) {
    verdict.flags |= kClamped;
  }
  verdict.velocity = gate(limited, in, verdict.flags);
  last_ = verdict.velocity;
  last_creep_ = (verdict.flags & kCreep) != 0 && last_.linear.norm() > kMoving;
  return verdict;
}

}  // namespace argus::node

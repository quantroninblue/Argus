#include "argus/node/motion.hpp"

#include <algorithm>
#include <cmath>

namespace argus::node {

using capabilities::AxisCapability;
using capabilities::MotionSpace;

namespace {

double clamp_axis(double v, const AxisCapability& a) { return std::clamp(v, a.min_mps, a.max_mps); }

double approach(double from, double to, double accel, double decel, double dt) {
  const bool slowing = from * to >= 0.0 && std::abs(to) < std::abs(from);
  const double rate = slowing ? decel : accel;
  if (rate <= 0.0) {
    return slowing ? to : from;
  }
  const double step = rate * dt;
  return from + std::clamp(to - from, -step, step);
}

}  // namespace

Twist MotionModel::clamp(const Twist& in) const {
  Twist v;
  v.linear.x() = clamp_axis(in.linear.x(), m_.x);
  v.linear.y() = m_.can_strafe ? clamp_axis(in.linear.y(), m_.y) : 0.0;
  v.linear.z() = m_.space == MotionSpace::Aerial ? clamp_axis(in.linear.z(), m_.z) : 0.0;
  double w = std::clamp(in.yaw_rate, -m_.yaw_rate_max_radps, m_.yaw_rate_max_radps);
  if (m_.min_turn_radius_m > 0.0) {
    const double limit = std::abs(v.linear.x()) / m_.min_turn_radius_m;
    w = std::clamp(w, -limit, limit);
  }
  v.yaw_rate = w;
  return v;
}

Twist MotionModel::accelerate(const Twist& from, const Twist& to, double dt) const {
  Twist v;
  v.linear.x() = approach(from.linear.x(), to.linear.x(), m_.x.accel_mps2, m_.x.decel_mps2, dt);
  v.linear.y() = approach(from.linear.y(), to.linear.y(), m_.y.accel_mps2, m_.y.decel_mps2, dt);
  v.linear.z() = approach(from.linear.z(), to.linear.z(), m_.z.accel_mps2, m_.z.decel_mps2, dt);
  v.yaw_rate = approach(from.yaw_rate, to.yaw_rate, m_.yaw_accel_radps2, m_.yaw_accel_radps2, dt);
  return v;
}

Pose4 MotionModel::integrate(const Pose4& pose, const Twist& v, double dt) {
  Pose4 out = pose;
  Pose4 mid = pose;
  mid.yaw = pose.yaw + 0.5 * v.yaw_rate * dt;
  out.position += mid.rotate(v.linear) * dt;
  out.yaw = wrap_angle(pose.yaw + v.yaw_rate * dt);
  return out;
}

double MotionModel::time_to_stop(const Twist& v) const {
  double t = 0.0;
  auto need = [&](double speed, double decel) {
    if (speed != 0.0) {
      t = std::max(t, decel > 0.0 ? std::abs(speed) / decel : 1e9);
    }
  };
  need(v.linear.x(), m_.x.decel_mps2);
  need(v.linear.y(), m_.y.decel_mps2);
  need(v.linear.z(), m_.z.decel_mps2);
  need(v.yaw_rate, m_.yaw_accel_radps2);
  return t;
}

std::vector<Pose4> MotionModel::stopping_path(const Pose4& start, const Twist& v, double coast_s,
                                              double spacing_m, double radius_m,
                                              std::vector<double>* times) const {
  std::vector<Pose4> poses = {start};
  if (times) {
    times->assign(1, 0.0);
  }
  if (v.is_zero()) {
    return poses;
  }
  const double brake_s = time_to_stop(v);
  const double total = coast_s + brake_s;
  const double speed = v.linear.norm();
  // Step so neither the base nor the body's rim moves more than the spacing.
  const double rim = std::abs(v.yaw_rate) * std::max(radius_m, 0.1);
  const double dt = std::min(0.05, spacing_m / std::max(speed + rim, 1e-6));
  // Each axis brakes at its own rate, as the controller (and accelerate()) does when sent a
  // stop: a body that brakes hard forward but softly sideways stops its forward motion first.
  auto axis_brake = [](double speed, double decel) {
    return speed == 0.0 ? 0.0 : (decel > 0.0 ? std::abs(speed) / decel : 1e9);
  };
  const double bx = axis_brake(v.linear.x(), m_.x.decel_mps2);
  const double by = axis_brake(v.linear.y(), m_.y.decel_mps2);
  const double bz = axis_brake(v.linear.z(), m_.z.decel_mps2);
  const double bw = axis_brake(v.yaw_rate, m_.yaw_accel_radps2);
  auto scale = [&](double mid, double b) {
    return mid < coast_s ? 1.0 : (b <= 0.0 ? 0.0 : std::max(0.0, 1.0 - (mid - coast_s) / b));
  };
  Pose4 pose = start;
  for (double t = 0.0; t < total - 1e-12; t += dt) {
    const double step = std::min(dt, total - t);
    const double mid = t + 0.5 * step;
    Twist now = v;
    now.linear.x() *= scale(mid, bx);
    now.linear.y() *= scale(mid, by);
    now.linear.z() *= scale(mid, bz);
    now.yaw_rate *= scale(mid, bw);
    pose = integrate(pose, now, step);
    poses.push_back(pose);
    if (times) {
      times->push_back(t + step);
    }
  }
  return poses;
}

}  // namespace argus::node

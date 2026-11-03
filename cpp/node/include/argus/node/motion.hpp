#pragma once

#include <vector>

#include "argus/capabilities/capabilities.hpp"
#include "argus/node/types.hpp"

namespace argus::node {

// How the body can move, from its motion capability. Used for planning rollouts and for the
// supervisor's stopping envelope.
class MotionModel {
 public:
  explicit MotionModel(const capabilities::MotionCapability& motion) : m_(motion) {}

  // Nearest velocity the body allows: axis and yaw limits, no strafing or climbing where the
  // body cannot, and yaw rate bounded by the minimum turn radius.
  Twist clamp(const Twist& v) const;

  // Velocity after `dt` of moving from `from` toward `to` at the axis acceleration limits.
  Twist accelerate(const Twist& from, const Twist& to, double dt) const;

  static Pose4 integrate(const Pose4& pose, const Twist& v, double dt);

  // Time to brake to rest from `v`, every axis slowing together.
  double time_to_stop(const Twist& v) const;

  // Poses the body passes through if it holds `v` for `coast_s` and then brakes to rest, no
  // more than `spacing_m` apart. Starts at `start`. With `times`, also when it reaches each
  // pose, from the start.
  std::vector<Pose4> stopping_path(const Pose4& start, const Twist& v, double coast_s,
                                   double spacing_m, double radius_m,
                                   std::vector<double>* times = nullptr) const;

  const capabilities::MotionCapability& capability() const { return m_; }

 private:
  capabilities::MotionCapability m_;
};

}  // namespace argus::node

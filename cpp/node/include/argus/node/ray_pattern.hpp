#pragma once

#include <vector>

#include "argus/capabilities/profile.hpp"
#include "argus/node/types.hpp"

namespace argus::node {

struct RayPatternOptions {
  double range_2d_step_deg = 1.0;
  double range_3d_h_step_deg = 2.0;
  double range_3d_v_step_deg = 4.0;
  int depth_cols = 40;
  int depth_rows = 30;
};

// Ray directions of a range sensor, as unit vectors in the sensor frame (x forward, z up).
// Azimuth runs across columns and elevation down rows, each sampled at cell centres.
struct RayPattern {
  int rows = 0;
  int cols = 0;
  std::vector<Vec3> directions;

  size_t size() const { return directions.size(); }
};

RayPattern make_ray_pattern(const capabilities::SensorSpec& sensor, const RayPatternOptions& options);

// Distance along each ray, from the sensor origin, at which it meets the robot's own body.
// inf where the ray leaves the body unobstructed. `mount` is the sensor pose in the base frame.
std::vector<double> self_hit_distances(const std::vector<capabilities::Shape>& body,
                                       const Eigen::Isometry3d& mount, const RayPattern& pattern,
                                       double max_range_m);

}  // namespace argus::node

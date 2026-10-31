#include "argus/node/ray_pattern.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus::node {

using capabilities::SensorKind;

namespace {

constexpr double kDeg = M_PI / 180.0;

int count(double span_deg, double step_deg) {
  return std::max(1, static_cast<int>(std::ceil(span_deg / step_deg - 1e-9)));
}

}  // namespace

RayPattern make_ray_pattern(const capabilities::SensorSpec& sensor, const RayPatternOptions& options) {
  RayPattern pattern;
  const double h = sensor.fov_h_deg.value_or(0.0);
  const double v = sensor.fov_v_deg.value_or(0.0);
  switch (sensor.kind) {
    case SensorKind::Range2d:
      pattern.rows = 1;
      pattern.cols = count(h, options.range_2d_step_deg);
      break;
    case SensorKind::Range3d:
      pattern.rows = std::max(2, count(v, options.range_3d_v_step_deg));
      pattern.cols = count(h, options.range_3d_h_step_deg);
      break;
    case SensorKind::Depth:
      pattern.rows = options.depth_rows;
      pattern.cols = options.depth_cols;
      break;
    default:
      return pattern;
  }
  const double v_span = sensor.kind == SensorKind::Range2d ? 0.0 : v;
  for (int r = 0; r < pattern.rows; ++r) {
    const double el = (v_span / 2.0 - (r + 0.5) * v_span / pattern.rows) * kDeg;
    for (int c = 0; c < pattern.cols; ++c) {
      const double az = (h / 2.0 - (c + 0.5) * h / pattern.cols) * kDeg;
      pattern.directions.emplace_back(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
                                      std::sin(el));
    }
  }
  return pattern;
}

std::vector<double> self_hit_distances(const std::vector<capabilities::Shape>& body,
                                       const Eigen::Isometry3d& mount, const RayPattern& pattern,
                                       double max_range_m) {
  const Vec3 origin = mount.translation();
  std::vector<const capabilities::Shape*> occluders;
  for (const auto& shape : body) {
    // A sensor mounted inside a shape looks out through it; that shape cannot block it.
    if (!shape.contains(origin)) {
      occluders.push_back(&shape);
    }
  }
  std::vector<double> distances(pattern.size(), std::numeric_limits<double>::infinity());
  for (size_t i = 0; i < pattern.size(); ++i) {
    const Vec3 dir = mount.linear() * pattern.directions[i];
    for (const auto* shape : occluders) {
      if (auto t = shape->intersect(origin, dir, 1e-6, max_range_m)) {
        distances[i] = std::min(distances[i], *t);
      }
    }
  }
  return distances;
}

}  // namespace argus::node

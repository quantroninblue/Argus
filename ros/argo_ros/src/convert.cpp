#include "argus/argo_ros/convert.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus::argo_ros {

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr double kDeg = M_PI / 180.0;

// Nearest return wins; with none, "no return" if any ray saw nothing within range.
struct Nearest {
  float best = kInf;
  bool hit = false;
  bool open = false;

  void hit_at(float r) {
    best = std::min(best, r);
    hit = true;
  }
  float value() const { return hit ? best : (open ? kInf : kNaN); }
};

}  // namespace

std::vector<float> scan_to_pattern(const Scan& scan, const capabilities::SensorSpec& sensor,
                                   const node::RayPattern& pattern) {
  std::vector<float> out(pattern.size(), kNaN);
  if (pattern.rows != 1 || scan.angle_increment == 0.0 || scan.ranges.empty()) {
    return out;
  }
  const double h = sensor.fov_h_deg.value_or(0.0) * kDeg;
  const double width = h / pattern.cols;
  const double inc = scan.angle_increment;
  const int n = static_cast<int>(scan.ranges.size());
  for (int c = 0; c < pattern.cols; ++c) {
    const double az = h / 2.0 - (c + 0.5) * width;
    // Beams whose angle falls in this column's patch.
    double i0 = (az - width / 2.0 - scan.angle_min) / inc;
    double i1 = (az + width / 2.0 - scan.angle_min) / inc;
    if (i0 > i1) std::swap(i0, i1);
    const int lo = std::max(0, static_cast<int>(std::ceil(i0)));
    const int hi = std::min(n - 1, static_cast<int>(std::floor(i1)));
    Nearest near;
    for (int i = lo; i <= hi; ++i) {
      const float r = scan.ranges[i];
      if (std::isnan(r) || r < scan.range_min) {
        continue;
      }
      if (std::isinf(r) || r > scan.range_max) {
        near.open = true;
      } else {
        near.hit_at(r);
      }
    }
    out[c] = near.value();
  }
  return out;
}

std::vector<float> points_to_pattern(const std::vector<float>& xyz, const capabilities::SensorSpec& sensor,
                                     const node::RayPattern& pattern) {
  std::vector<float> out(pattern.size(), kNaN);
  if (pattern.rows < 1 || pattern.cols < 1) {
    return out;
  }
  const double h = sensor.fov_h_deg.value_or(0.0) * kDeg;
  const double v = sensor.fov_v_deg.value_or(0.0) * kDeg;
  const double min_r = sensor.range_m ? sensor.range_m->min : 0.0;
  const double max_r = sensor.range_m ? sensor.range_m->max : std::numeric_limits<double>::infinity();
  std::vector<float> best(pattern.size(), kInf);
  for (size_t k = 0; k + 2 < xyz.size(); k += 3) {
    const double x = xyz[k], y = xyz[k + 1], z = xyz[k + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    const double r = std::sqrt(x * x + y * y + z * z);
    if (r < min_r || r > max_r) {
      continue;
    }
    const double az = std::atan2(y, x);
    const double el = std::atan2(z, std::hypot(x, y));
    const int c = static_cast<int>(std::floor((h / 2.0 - az) / (h / pattern.cols)));
    const int row = pattern.rows == 1 ? 0 : static_cast<int>(std::floor((v / 2.0 - el) / (v / pattern.rows)));
    if (c < 0 || c >= pattern.cols || row < 0 || row >= pattern.rows) {
      continue;
    }
    float& b = best[static_cast<size_t>(row) * pattern.cols + c];
    b = std::min(b, static_cast<float>(r));
  }
  for (size_t i = 0; i < best.size(); ++i) {
    if (std::isfinite(best[i])) {
      out[i] = best[i];
    }
  }
  return out;
}

std::vector<float> depth_to_pattern(const DepthImage& image, const capabilities::SensorSpec& sensor,
                                    const node::RayPattern& pattern) {
  std::vector<float> out(pattern.size(), kNaN);
  if (image.width <= 0 || image.height <= 0 || image.fx <= 0.0 || image.fy <= 0.0 ||
      image.depth.size() < static_cast<size_t>(image.width) * image.height) {
    return out;
  }
  const double h = sensor.fov_h_deg.value_or(0.0) * kDeg;
  const double v = sensor.fov_v_deg.value_or(0.0) * kDeg;
  // Half a pattern patch, in pixels.
  const int half_u = std::max(0, static_cast<int>(std::lround(0.5 * image.fx * h / pattern.cols)));
  const int half_v = std::max(0, static_cast<int>(std::lround(0.5 * image.fy * v / std::max(1, pattern.rows))));
  for (size_t i = 0; i < pattern.size(); ++i) {
    const node::Vec3& d = pattern.directions[i];
    if (d.x() <= 1e-6) {
      continue;
    }
    // Sensor frame (x forward, y left, z up) to the optical frame (z forward, x right, y down).
    const double u = image.fx * (-d.y() / d.x()) + image.cx;
    const double w = image.fy * (-d.z() / d.x()) + image.cy;
    const int u0 = static_cast<int>(std::lround(u));
    const int v0 = static_cast<int>(std::lround(w));
    if (u0 < 0 || u0 >= image.width || v0 < 0 || v0 >= image.height) {
      continue;
    }
    Nearest near;
    for (int pv = std::max(0, v0 - half_v); pv <= std::min(image.height - 1, v0 + half_v); ++pv) {
      for (int pu = std::max(0, u0 - half_u); pu <= std::min(image.width - 1, u0 + half_u); ++pu) {
        const float z = image.depth[static_cast<size_t>(pv) * image.width + pu];
        if (std::isnan(z) || z <= 0.0f) {
          continue;
        }
        if (std::isinf(z)) {
          near.open = true;
        } else {
          near.hit_at(static_cast<float>(z / d.x()));
        }
      }
    }
    out[i] = near.value();
  }
  return out;
}

double yaw_of(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace argus::argo_ros

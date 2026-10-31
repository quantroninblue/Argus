#include "argus/node/view.hpp"

#include <algorithm>
#include <cmath>

namespace argus::node {


namespace {

// Fractional index of `angle` in a span of `count` samples centred on cells, with sample 0 at
// +span/2 and the last at -span/2 (as `make_ray_pattern` lays them out). More than half a step
// outside the span is out of view.
bool bracket(double angle, double span, int count, bool wraps, int& lo, int& hi) {
  const double step = span / count;
  double f = (span / 2.0 - angle) / step - 0.5;
  if (wraps) {
    f = std::fmod(f, static_cast<double>(count));
    if (f < 0.0) {
      f += count;
    }
    lo = static_cast<int>(std::floor(f)) % count;
    hi = (lo + 1) % count;
    return true;
  }
  if (f < -0.5 || f > count - 0.5) {
    return false;
  }
  lo = std::clamp(static_cast<int>(std::floor(f)), 0, count - 1);
  hi = std::clamp(lo + 1, 0, count - 1);
  if (f < 0.0) {
    hi = lo;
  }
  return true;
}

}  // namespace

bool RangeView::sees(const Vec3& point_odom) const {
  const ViewLayout& L = *layout;
  const Vec3 q = odom_to_sensor * point_odom;
  const double d = q.norm();
  if (d < std::max(L.min_range_m, 1e-6) || L.cols < 1) {
    return false;
  }
  int r0 = 0;
  int r1 = 0;
  if (L.rows == 1) {
    // A scan sees one plane; the point's cell must cross it.
    if (std::abs(q.z()) > 0.5 * cell_m) {
      return false;
    }
  } else if (!bracket(std::atan2(q.z(), std::hypot(q.x(), q.y())), L.fov_v_rad, L.rows, false, r0, r1)) {
    return false;
  }
  int c0 = 0;
  int c1 = 0;
  const bool wraps = L.fov_h_rad >= 2.0 * M_PI - 1e-6;
  if (!bracket(std::atan2(q.y(), q.x()), L.fov_h_rad, L.cols, wraps, c0, c1)) {
    return false;
  }
  // Short of every ray around the point by a cell, so the cell an obstacle was hit in is never
  // taken for space that was looked through.
  const int rows[2] = {r0, r1};
  const int cols[2] = {c0, c1};
  for (int r : rows) {
    for (int c : cols) {
      const float len = (*free_m)[static_cast<size_t>(r) * L.cols + c];
      if (!(d <= len - cell_m)) {
        return false;  // NaN fails too: that ray said nothing
      }
    }
  }
  return true;
}

namespace {

template <typename Sees>
void mark_near(LocalMap& map, const Vec3& center, double radius_m, double z_low, double z_high, Sees sees) {
  const double res = map.resolution();
  const Vec3 lo_p(center.x() - radius_m, center.y() - radius_m, z_low);
  const Vec3 hi_p(center.x() + radius_m, center.y() + radius_m, z_high);
  const Index3 a = map.index_of(lo_p);
  const Index3 b = map.index_of(hi_p);
  const double r2 = (radius_m + res) * (radius_m + res);
  for (int ix = a.x(); ix <= b.x(); ++ix) {
    for (int iy = a.y(); iy <= b.y(); ++iy) {
      for (int iz = a.z(); iz <= b.z(); ++iz) {
        const Index3 i(ix, iy, iz);
        if (!map.contains(i) || map.viewed(i) || map.cell(i) != Cell::Unknown) {
          continue;
        }
        const Vec3 c = map.center_of(i);
        const Vec2 d = (c - center).head<2>();
        if (d.squaredNorm() > r2) {
          continue;
        }
        if (sees(c)) {
          map.mark_viewed(i);
        }
      }
    }
  }
}

}  // namespace

void mark_viewed(LocalMap& map, const RangeView& view, const Vec3& center, double radius_m, double z_low,
                 double z_high) {
  mark_near(map, center, radius_m, z_low, z_high, [&view](const Vec3& c) { return view.sees(c); });
}

void mark_zone_viewed(LocalMap& map, const Vec3& center, double radius_m, double z_low, double z_high) {
  mark_near(map, center, radius_m, z_low, z_high, [](const Vec3&) { return true; });
}

}  // namespace argus::node

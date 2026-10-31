#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "argus/node/local_map.hpp"
#include "argus/node/types.hpp"

namespace argus::node {

// Ray layout of one range sensor, matching `make_ray_pattern`: azimuth across columns and
// elevation down rows, sampled at cell centres. A scan (one row) sees a plane.
struct ViewLayout {
  int rows = 1;
  int cols = 0;
  double fov_h_rad = 0.0;
  double fov_v_rad = 0.0;
  double min_range_m = 0.0;
};

// One range frame as a view: where the sensor was and how far each ray saw free space (its
// hit range, or the maximum range for a no return counted as free; NaN where the ray says
// nothing).
struct RangeView {
  const ViewLayout* layout = nullptr;
  Eigen::Isometry3d odom_to_sensor = Eigen::Isometry3d::Identity();
  const std::vector<float>* free_m = nullptr;
  double cell_m = 0.1;

  // Whether the point lies inside the view: beyond the sensor's minimum range and short, by a
  // cell, of every ray around it. Cells between rays pass; cells outside the field of view,
  // too close, or behind a hit do not.
  bool sees(const Vec3& point_odom) const;
};

// Marks as viewed every unknown cell of `map` within `radius_m` of `center` horizontally and
// between `z_low` and `z_high` (the height band the body can hit) that the view sees.
//
// The map only clears cells a ray passes through, so cells between sparse rays stay unknown
// although the sensor looked straight at them. Projecting each nearby cell into the frame,
// rather than tracing rays out to it, is the projective integration of wavemap (Reijgwart et
// al. 2023) and nvblox; here it only records that the cell was looked at.
void mark_viewed(LocalMap& map, const RangeView& view, const Vec3& center, double radius_m, double z_low,
                 double z_high);

// Marks the whole zone as viewed: the profile's declared clear space where Argo starts.
void mark_zone_viewed(LocalMap& map, const Vec3& center, double radius_m, double z_low, double z_high);

}  // namespace argus::node

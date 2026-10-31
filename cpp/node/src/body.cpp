#include "argus/node/body.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus::node {

using capabilities::MotionSpace;

bool inside_convex(const std::vector<Vec2>& poly, const Vec2& p) {
  const size_t n = poly.size();
  if (n < 3) {
    return false;
  }
  for (size_t i = 0; i < n; ++i) {
    const Vec2 e = poly[(i + 1) % n] - poly[i];
    const Vec2 q = p - poly[i];
    if (e.x() * q.y() - e.y() * q.x() < 0.0) {
      return false;
    }
  }
  return true;
}

double distance_to_convex(const std::vector<Vec2>& poly, const Vec2& p) {
  if (inside_convex(poly, p)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  const size_t n = poly.size();
  for (size_t i = 0; i < n; ++i) {
    const Vec2 a = poly[i];
    const Vec2 e = poly[(i + 1) % n] - a;
    const double len2 = e.squaredNorm();
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(e) / len2, 0.0, 1.0) : 0.0;
    best = std::min(best, (a + t * e - p).norm());
  }
  return best;
}

std::vector<Vec2> place(const std::vector<Vec2>& poly, const Pose4& pose) {
  std::vector<Vec2> out;
  out.reserve(poly.size());
  for (const Vec2& v : poly) {
    out.push_back(pose.to_odom(Vec3(v.x(), v.y(), 0.0)).head<2>());
  }
  return out;
}

BodyVolume::BodyVolume(const capabilities::Capabilities& caps, double ground_z_m,
                       double min_obstacle_height_m, double max_unknown_fraction)
    : planar_(caps.motion.space == MotionSpace::Planar),
      footprint_(caps.collision.footprint),
      body_footprint_(caps.collision.body_footprint),
      radius_(caps.collision.bounding_radius_m),
      body_radius_(caps.collision.body_radius_m),
      max_unknown_fraction_(max_unknown_fraction) {
  z_low_ = ground_z_m + min_obstacle_height_m;
  z_high_ = caps.collision.aabb_max.z() + caps.collision.padding_m;
  inscribed_ = caps.collision.inscribed_radius_m;
}

Cell BodyVolume::column(const LocalMap& map, int ix, int iy, int z0, int z1) const {
  bool free = false;
  for (int iz = z0; iz <= z1; ++iz) {
    const Cell c = map.cell(Index3(ix, iy, iz));
    if (c == Cell::Occupied) {
      return Cell::Occupied;
    }
    free = free || c == Cell::Free;
  }
  return free ? Cell::Free : Cell::Unknown;
}

Contact BodyVolume::contact(const LocalMap& map, const Pose4& pose, const Pose4* here) const {
  return planar_ ? planar_contact(map, pose, here) : aerial_contact(map, pose, here);
}

Contact BodyVolume::planar_contact(const LocalMap& map, const Pose4& pose, const Pose4* here) const {
  const double res = map.resolution();
  const double slack = res * M_SQRT1_2;
  const std::vector<Vec2> poly = place(footprint_, pose);
  const std::vector<Vec2> body = place(body_footprint_, pose);
  std::vector<Vec2> exempt;
  if (here) {
    exempt = place(footprint_, *here);
  }
  Vec2 lo = poly[0];
  Vec2 hi = poly[0];
  for (const Vec2& v : poly) {
    lo = lo.cwiseMin(v);
    hi = hi.cwiseMax(v);
  }
  const Index3 a = map.index_of(Vec3(lo.x() - slack, lo.y() - slack, pose.position.z() + z_low_));
  const Index3 b = map.index_of(Vec3(hi.x() + slack, hi.y() + slack, pose.position.z() + z_high_));
  int total = 0;
  int unknown = 0;
  for (int ix = a.x(); ix <= b.x(); ++ix) {
    for (int iy = a.y(); iy <= b.y(); ++iy) {
      const Vec2 c = map.center_of(Index3(ix, iy, 0)).head<2>();
      if (distance_to_convex(poly, c) > slack) {
        continue;
      }
      ++total;
      const Cell state = column(map, ix, iy, a.z(), b.z());
      const bool here_cell = here && distance_to_convex(exempt, c) <= slack;
      if (state == Cell::Occupied) {
        if (here_cell && distance_to_convex(body, c) > slack) {
          continue;
        }
        return Contact::Occupied;
      }
      if (state == Cell::Unknown && !here_cell) {
        ++unknown;
      }
    }
  }
  if (total > 0 && unknown > max_unknown_fraction_ * total) {
    return Contact::Unknown;
  }
  return Contact::Free;
}

Contact BodyVolume::aerial_contact(const LocalMap& map, const Pose4& pose, const Pose4* here) const {
  const double res = map.resolution();
  const double slack = res * 0.87;
  const Vec3 p = pose.position;
  const Index3 a = map.index_of(p - Vec3::Constant(radius_ + slack));
  const Index3 b = map.index_of(p + Vec3::Constant(radius_ + slack));
  int total = 0;
  int unknown = 0;
  for (int ix = a.x(); ix <= b.x(); ++ix) {
    for (int iy = a.y(); iy <= b.y(); ++iy) {
      for (int iz = a.z(); iz <= b.z(); ++iz) {
        const Index3 i(ix, iy, iz);
        const Vec3 c = map.center_of(i);
        const double d = (c - p).norm();
        if (d > radius_ + slack) {
          continue;
        }
        ++total;
        const Cell state = map.cell(i);
        const bool here_cell = here && (c - here->position).norm() <= radius_ + slack;
        if (state == Cell::Occupied) {
          if (here_cell && d > body_radius_ + slack) {
            continue;
          }
          return Contact::Occupied;
        }
        if (state == Cell::Unknown && !here_cell) {
          ++unknown;
        }
      }
    }
  }
  if (total > 0 && unknown > max_unknown_fraction_ * total) {
    return Contact::Unknown;
  }
  return Contact::Free;
}

}  // namespace argus::node

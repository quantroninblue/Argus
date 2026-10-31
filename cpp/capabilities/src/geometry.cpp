#include "argus/capabilities/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus::capabilities {

Eigen::Isometry3d Pose::transform() const {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = xyz;
  t.linear() = (Eigen::AngleAxisd(rpy.z(), Vec3::UnitZ()) *
                Eigen::AngleAxisd(rpy.y(), Vec3::UnitY()) *
                Eigen::AngleAxisd(rpy.x(), Vec3::UnitX()))
                   .toRotationMatrix();
  return t;
}

bool Shape::contains(const Vec3& point) const {
  const Vec3 local = pose.transform().inverse() * point;
  switch (type) {
    case ShapeType::Box:
      return std::abs(local.x()) <= 0.5 * size.x() && std::abs(local.y()) <= 0.5 * size.y() &&
             std::abs(local.z()) <= 0.5 * size.z();
    case ShapeType::Cylinder:
      return local.head<2>().norm() <= radius && std::abs(local.z()) <= 0.5 * length;
    case ShapeType::Sphere:
      return local.norm() <= radius;
  }
  return false;
}

namespace {

void keep_if_better(double t, double t_min, double t_max, std::optional<double>& best) {
  if (t > t_min && t < t_max && (!best || t < *best)) {
    best = t;
  }
}

std::optional<double> intersect_box(const Vec3& o, const Vec3& d, const Vec3& half, double t_min,
                                    double t_max) {
  double enter = -std::numeric_limits<double>::infinity();
  double exit = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    if (std::abs(d[i]) < 1e-12) {
      if (std::abs(o[i]) > half[i]) {
        return std::nullopt;
      }
      continue;
    }
    double t0 = (-half[i] - o[i]) / d[i];
    double t1 = (half[i] - o[i]) / d[i];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    enter = std::max(enter, t0);
    exit = std::min(exit, t1);
    if (enter > exit) {
      return std::nullopt;
    }
  }
  std::optional<double> best;
  keep_if_better(enter, t_min, t_max, best);
  keep_if_better(exit, t_min, t_max, best);
  return best;
}

std::optional<double> intersect_sphere(const Vec3& o, const Vec3& d, double r, double t_min,
                                       double t_max) {
  const double a = d.squaredNorm();
  const double b = 2.0 * o.dot(d);
  const double c = o.squaredNorm() - r * r;
  const double disc = b * b - 4.0 * a * c;
  if (a < 1e-18 || disc < 0.0) {
    return std::nullopt;
  }
  const double root = std::sqrt(disc);
  std::optional<double> best;
  keep_if_better((-b - root) / (2.0 * a), t_min, t_max, best);
  keep_if_better((-b + root) / (2.0 * a), t_min, t_max, best);
  return best;
}

std::optional<double> intersect_cylinder(const Vec3& o, const Vec3& d, double r, double len,
                                         double t_min, double t_max) {
  const double half = 0.5 * len;
  std::optional<double> best;
  const double a = d.x() * d.x() + d.y() * d.y();
  if (a > 1e-18) {
    const double b = 2.0 * (o.x() * d.x() + o.y() * d.y());
    const double c = o.x() * o.x() + o.y() * o.y() - r * r;
    const double disc = b * b - 4.0 * a * c;
    if (disc >= 0.0) {
      const double root = std::sqrt(disc);
      for (double t : {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)}) {
        if (std::abs(o.z() + t * d.z()) <= half) {
          keep_if_better(t, t_min, t_max, best);
        }
      }
    }
  }
  if (std::abs(d.z()) > 1e-12) {
    for (double cap : {-half, half}) {
      const double t = (cap - o.z()) / d.z();
      const Vec3 p = o + t * d;
      if (p.x() * p.x() + p.y() * p.y() <= r * r) {
        keep_if_better(t, t_min, t_max, best);
      }
    }
  }
  return best;
}

}  // namespace

std::optional<double> Shape::intersect(const Vec3& origin, const Vec3& direction, double t_min,
                                       double t_max) const {
  const Eigen::Isometry3d inv = pose.transform().inverse();
  const Vec3 o = inv * origin;
  const Vec3 d = inv.linear() * direction;
  switch (type) {
    case ShapeType::Box:
      return intersect_box(o, d, 0.5 * size, t_min, t_max);
    case ShapeType::Cylinder:
      return intersect_cylinder(o, d, radius, length, t_min, t_max);
    case ShapeType::Sphere:
      return intersect_sphere(o, d, radius, t_min, t_max);
  }
  return std::nullopt;
}

std::vector<Vec3> Shape::surface_points(int circle_segments) const {
  const Eigen::Isometry3d t = pose.transform();
  std::vector<Vec3> points;
  switch (type) {
    case ShapeType::Box:
      for (double sx : {-0.5, 0.5}) {
        for (double sy : {-0.5, 0.5}) {
          for (double sz : {-0.5, 0.5}) {
            points.push_back(t * Vec3(sx * size.x(), sy * size.y(), sz * size.z()));
          }
        }
      }
      break;
    case ShapeType::Cylinder:
      for (int i = 0; i < circle_segments; ++i) {
        const double a = 2.0 * M_PI * i / circle_segments;
        for (double sz : {-0.5, 0.5}) {
          points.push_back(t * Vec3(radius * std::cos(a), radius * std::sin(a), sz * length));
        }
      }
      break;
    case ShapeType::Sphere: {
      // The footprint only needs the horizontal extent, the poles give the vertical one.
      const Vec3 c = t.translation();
      for (int i = 0; i < circle_segments; ++i) {
        const double a = 2.0 * M_PI * i / circle_segments;
        points.push_back(c + Vec3(radius * std::cos(a), radius * std::sin(a), 0.0));
      }
      points.push_back(c + Vec3(0.0, 0.0, radius));
      points.push_back(c - Vec3(0.0, 0.0, radius));
      break;
    }
  }
  return points;
}

std::vector<Vec2> convex_hull(std::vector<Vec2> points) {
  std::sort(points.begin(), points.end(), [](const Vec2& a, const Vec2& b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  points.erase(std::unique(points.begin(), points.end(),
                           [](const Vec2& a, const Vec2& b) { return (a - b).norm() < 1e-12; }),
               points.end());
  if (points.size() < 3) {
    return points;
  }
  auto cross = [](const Vec2& o, const Vec2& a, const Vec2& b) {
    return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
  };
  std::vector<Vec2> hull(2 * points.size());
  size_t k = 0;
  for (const Vec2& p : points) {
    while (k >= 2 && cross(hull[k - 2], hull[k - 1], p) <= 1e-12) {
      --k;
    }
    hull[k++] = p;
  }
  const size_t lower = k + 1;
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (k >= lower && cross(hull[k - 2], hull[k - 1], *it) <= 1e-12) {
      --k;
    }
    hull[k++] = *it;
  }
  hull.resize(k - 1);
  return hull;
}

double ray_exit_distance(const std::vector<Vec2>& polygon, const Vec2& direction) {
  double best = 0.0;
  const size_t n = polygon.size();
  for (size_t i = 0; i < n; ++i) {
    const Vec2& a = polygon[i];
    const Vec2& b = polygon[(i + 1) % n];
    const Vec2 edge = b - a;
    const double denom = direction.x() * edge.y() - direction.y() * edge.x();
    if (std::abs(denom) < 1e-12) {
      continue;
    }
    const double t = (a.x() * edge.y() - a.y() * edge.x()) / denom;
    const double s = (a.x() * direction.y() - a.y() * direction.x()) / denom;
    if (t >= 0.0 && s >= -1e-9 && s <= 1.0 + 1e-9) {
      best = std::max(best, t);
    }
  }
  return best;
}

}  // namespace argus::capabilities

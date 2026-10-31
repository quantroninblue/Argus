#pragma once

#include <optional>
#include <vector>

#include <Eigen/Geometry>

namespace argus::capabilities {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;

// Position and fixed-axis roll/pitch/yaw in radians, same convention as URDF.
struct Pose {
  Vec3 xyz = Vec3::Zero();
  Vec3 rpy = Vec3::Zero();

  Eigen::Isometry3d transform() const;
};

enum class ShapeType { Box, Cylinder, Sphere };

// Collision shape in the base frame. Cylinders run along their local z axis.
struct Shape {
  ShapeType type = ShapeType::Box;
  Pose pose;
  Vec3 size = Vec3::Zero();
  double radius = 0.0;
  double length = 0.0;

  bool contains(const Vec3& point) const;

  // Smallest t in (t_min, t_max) where origin + t * direction meets the surface.
  std::optional<double> intersect(const Vec3& origin, const Vec3& direction, double t_min,
                                  double t_max) const;

  // Points on the shape's outer surface, enough to bound it and project a footprint.
  std::vector<Vec3> surface_points(int circle_segments = 16) const;
};

// Counter-clockwise convex hull without collinear points.
std::vector<Vec2> convex_hull(std::vector<Vec2> points);

// Distance from the origin to the far boundary of a convex polygon along a unit direction.
// Returns 0 if the ray never meets the polygon.
double ray_exit_distance(const std::vector<Vec2>& polygon, const Vec2& direction);

}  // namespace argus::capabilities

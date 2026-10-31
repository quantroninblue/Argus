#pragma once

#include <vector>

#include "argus/capabilities/capabilities.hpp"
#include "argus/node/local_map.hpp"
#include "argus/node/types.hpp"
#include "argus/node/view.hpp"

namespace argus::node {

enum class Contact { Free, Unknown, Occupied };

// The robot's collision volume placed in the local map. Planar bodies are a footprint extruded
// over the height band they can hit; aerial bodies are a sphere around the base origin.
class BodyVolume {
 public:
  BodyVolume(const capabilities::Capabilities& caps, double ground_z_m, double min_obstacle_height_m,
             double max_unknown_fraction);

  // Worst state among the cells the padded body covers at `pose`. A planar column counts as
  // free when no cell in the band is occupied and at least one was seen free.
  //
  // `here` is where the robot stands now. Unknown cells inside its padded volume there count
  // as free, and occupied ones only count if the body itself (unpadded) would reach them at
  // `pose`, so a robot already inside its padding near a wall can still move away.
  //
  // Unknown cells a sensor has looked at (between its rays, see view.hpp) count as free. Of
  // the cells no sensor has looked at, those only above or below every sensor's vertical view
  // on a bearing some sensor covers are accepted too, as PX4's collision prevention and
  // avoidance do (they work in the horizontal plane); the unknown share counts the rest:
  // cells beside or ahead of the body that no sensor covers, or closer than a sensor's
  // minimum range. A planar body is judged per column.
  Contact contact(const LocalMap& map, const Pose4& pose, const Pose4* here) const;

  // Range sensors, for telling space above or below their view from space beside the body.
  struct Sensor {
    Eigen::Isometry3d mount = Eigen::Isometry3d::Identity();
    ViewLayout layout;
    double max_range_m = 0.0;
  };
  void set_sensors(std::vector<Sensor> sensors, double cell_m) {
    sensors_ = std::move(sensors);
    map_cell_ = cell_m;
  }

  bool planar() const { return planar_; }
  // Height band, relative to the base origin, in which a planar body can hit something.
  double z_low_m() const { return z_low_; }
  double z_high_m() const { return z_high_; }
  const std::vector<Vec2>& footprint() const { return footprint_; }
  const std::vector<Vec2>& body_footprint() const { return body_footprint_; }
  double radius_m() const { return radius_; }
  double body_radius_m() const { return body_radius_; }
  // Distance from the base origin to the nearest padded footprint edge.
  double inscribed_radius_m() const { return inscribed_; }

 private:
  Cell column(const LocalMap& map, int ix, int iy, int z0, int z1) const;
  bool column_viewed(const LocalMap& map, int ix, int iy, int z0, int z1) const;
  // Some sensor covers the point's bearing, and none has it inside its field of view.
  bool out_of_vertical_view(const Pose4& pose, const Vec3& point, bool& covered, bool& in_view) const;
  bool column_out_of_vertical_view(const LocalMap& map, const Pose4& pose, int ix, int iy, int z0, int z1) const;
  Contact planar_contact(const LocalMap& map, const Pose4& pose, const Pose4* here) const;
  Contact aerial_contact(const LocalMap& map, const Pose4& pose, const Pose4* here) const;

  bool planar_ = true;
  double z_low_ = 0.0;
  double z_high_ = 0.0;
  std::vector<Vec2> footprint_;
  std::vector<Vec2> body_footprint_;
  double radius_ = 0.0;
  double body_radius_ = 0.0;
  double inscribed_ = 0.0;
  double max_unknown_fraction_ = 0.2;
  std::vector<Sensor> sensors_;
  double map_cell_ = 0.1;
};

// Polygon helpers for counter-clockwise convex polygons.
bool inside_convex(const std::vector<Vec2>& polygon, const Vec2& p);
double distance_to_convex(const std::vector<Vec2>& polygon, const Vec2& p);
std::vector<Vec2> place(const std::vector<Vec2>& polygon, const Pose4& pose);

}  // namespace argus::node

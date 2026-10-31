#pragma once

#include <vector>

#include "argus/capabilities/capabilities.hpp"
#include "argus/node/local_map.hpp"
#include "argus/node/types.hpp"

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
  Contact contact(const LocalMap& map, const Pose4& pose, const Pose4* here) const;

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
};

// Polygon helpers for counter-clockwise convex polygons.
bool inside_convex(const std::vector<Vec2>& polygon, const Vec2& p);
double distance_to_convex(const std::vector<Vec2>& polygon, const Vec2& p);
std::vector<Vec2> place(const std::vector<Vec2>& polygon, const Pose4& pose);

}  // namespace argus::node

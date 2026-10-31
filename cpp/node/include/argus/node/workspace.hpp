#pragma once

#include "argus/grid/search.hpp"
#include "argus/node/body.hpp"
#include "argus/node/local_map.hpp"

namespace argus::node {

using grid::distance_field;
using grid::PathRequest;
using grid::PathResult;
using grid::PathSearch;

// The local map reduced to what one body needs for planning, with a distance field. Planar
// bodies get a 2D grid of columns over their height band; aerial bodies the voxel grid.
class Workspace : public grid::SearchGrid {
 public:
  void build(const LocalMap& map, const BodyVolume& body, const Pose4& pose);
};

}  // namespace argus::node

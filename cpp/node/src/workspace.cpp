#include "argus/node/workspace.hpp"

#include <utility>

namespace argus::node {

void Workspace::build(const LocalMap& map, const BodyVolume& body, const Pose4& pose) {
  const bool planar = body.planar();
  const Index3 origin = map.window_min();
  const Index3& md = map.dims();
  const Index3 dims = planar ? Index3(md.x(), md.y(), 1) : md;
  auto linear = [&](const Index3& i) {
    return (static_cast<size_t>(i.x()) * dims.y() + i.y()) * dims.z() + i.z();
  };
  std::vector<Cell> cells(static_cast<size_t>(dims.prod()), Cell::Unknown);
  if (planar) {
    const int z0 = map.index_of(Vec3(0, 0, pose.position.z() + body.z_low_m())).z();
    const int z1 = map.index_of(Vec3(0, 0, pose.position.z() + body.z_high_m())).z();
    for (int x = 0; x < dims.x(); ++x) {
      for (int y = 0; y < dims.y(); ++y) {
        bool free = false;
        bool occ = false;
        for (int iz = z0; iz <= z1 && !occ; ++iz) {
          const Cell c = map.cell(Index3(origin.x() + x, origin.y() + y, iz));
          occ = c == Cell::Occupied;
          free = free || c == Cell::Free;
        }
        cells[linear(Index3(x, y, 0))] = occ ? Cell::Occupied : (free ? Cell::Free : Cell::Unknown);
      }
    }
  } else {
    for (int x = 0; x < dims.x(); ++x) {
      for (int y = 0; y < dims.y(); ++y) {
        for (int z = 0; z < dims.z(); ++z) {
          cells[linear(Index3(x, y, z))] = map.cell(origin + Index3(x, y, z));
        }
      }
    }
  }
  assign(origin, dims, map.resolution(), planar, pose.position.z(), std::move(cells));
}

}  // namespace argus::node

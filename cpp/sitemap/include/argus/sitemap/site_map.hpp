#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "argus/grid/evidence.hpp"
#include "argus/grid/trace.hpp"

namespace argus::sitemap {

using grid::Cell;
using grid::CellDelta;
using grid::Index3;
using grid::Vec3;

struct Tile {
  std::array<int8_t, grid::kTileCells> log_odds{};
  std::array<uint8_t, grid::kTileCells> observed{};
  uint64_t version = 0;  // site map version at the tile's last change
  double updated = 0.0;  // evidence stamp of the last change
};

// One body's 2D view of the map: each column over a height band. A column is occupied if any
// cell in the band is occupied, free if none is and at least one was seen free, unknown
// otherwise. Argo's local workspace uses the same rule.
struct PlanarView {
  Index3 origin = Index3::Zero();  // cell index of element (0, 0); z is unused
  int nx = 0;
  int ny = 0;
  double resolution = 0.1;
  std::vector<Cell> cells;  // cells[x * ny + y]

  Cell at(int x, int y) const { return cells[static_cast<size_t>(x) * ny + y]; }
};

// The persistent map of one site, in the site frame. Sparse: only tiles that received
// evidence exist. The map has an epoch; a reset starts a new one, and evidence recorded
// against an older epoch is refused so it cannot land in the wrong frame.
class SiteMap {
 public:
  explicit SiteMap(std::string site_id, double resolution_m = 0.1, uint64_t epoch = 1);

  const std::string& site_id() const { return site_id_; }
  uint64_t epoch() const { return epoch_; }
  double resolution() const { return res_; }
  // Increases with every change; tiles record the version that last changed them.
  uint64_t version() const { return version_; }
  size_t tile_count() const { return tiles_.size(); }

  void reset();

  // Adds evidence from Argo. Returns false and changes nothing if `epoch` is not current.
  bool apply(uint64_t epoch, const std::vector<CellDelta>& deltas, double stamp);
  // Integrates a range frame directly, with the same rules as Argo's local map.
  void integrate(const Vec3& origin, const std::vector<Vec3>& ends, const std::vector<bool>& hits,
                 double stamp);

  Cell cell(const Index3& index) const;
  Cell at(const Vec3& point) const { return cell(grid::index_of(point, res_)); }

  // Cell bounds of all tiles, `hi` exclusive. False when the map is empty.
  bool bounds(Index3& lo, Index3& hi) const;

  // Heights are in the site frame. Covers all tiles unless bounds are given (cells, hi exclusive).
  PlanarView planar_view(double z_low_m, double z_high_m) const;
  PlanarView planar_view(double z_low_m, double z_high_m, const Index3& lo, const Index3& hi) const;

  // 3D view at `factor` times the map resolution over cells [lo, hi) of the coarse grid (so
  // coarse cell c covers fine cells c * factor to c * factor + factor - 1). A coarse cell is
  // occupied if any of its cells is, free if any is free, unknown otherwise. Laid out like
  // grid::SearchGrid::linear.
  std::vector<Cell> coarse_view(int factor, const Index3& lo, const Index3& hi) const;

  // Tiles changed after `version`, for map deltas.
  std::vector<Index3> changed_since(uint64_t version) const;

  // Payload of MapDelta with MAP_ENCODING_VOXEL_TILES, also used to save a map, little-endian:
  //
  //   u32 tile_count
  //   per tile: i32 x, i32 y, i32 z, u64 version, f64 updated,
  //             kTileCells x i8 log-odds, kTileCells / 8 bytes observed bitmask
  std::string encode_tiles(const std::vector<Index3>& tiles) const;
  std::string encode_all() const;
  // Replaces tiles with the same index. Throws std::invalid_argument on a malformed payload,
  // in which case nothing is changed.
  void load_tiles(const std::string& payload);

 private:
  struct TileHash {
    size_t operator()(const Index3& t) const {
      return (static_cast<size_t>(t.x()) * 73856093u) ^ (static_cast<size_t>(t.y()) * 19349663u) ^
             (static_cast<size_t>(t.z()) * 83492791u);
    }
  };

  void update(const Index3& index, int delta, double stamp);

  std::string site_id_;
  double res_;
  uint64_t epoch_;
  uint64_t version_ = 0;
  std::unordered_map<Index3, Tile, TileHash> tiles_;
  grid::FrameTracer tracer_;
};

}  // namespace argus::sitemap

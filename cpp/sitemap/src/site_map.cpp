#include "argus/sitemap/site_map.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "argus/grid/wire.hpp"

namespace argus::sitemap {

using grid::kTileCells;
using grid::kTileSize;

SiteMap::SiteMap(std::string site_id, double resolution_m, uint64_t epoch)
    : site_id_(std::move(site_id)), res_(resolution_m), epoch_(epoch) {
  if (!(resolution_m > 0.0)) {
    throw std::invalid_argument("site map resolution must be positive");
  }
}

void SiteMap::reset() {
  tiles_.clear();
  ++epoch_;
  ++version_;
}

void SiteMap::update(const Index3& index, int delta, double stamp) {
  Tile& tile = tiles_[grid::tile_of(index)];
  const int k = grid::offset_in_tile(index);
  tile.log_odds[k] = grid::add_log_odds(tile.log_odds[k], delta);
  tile.observed[k] = 1;
  tile.version = version_;
  tile.updated = std::max(tile.updated, stamp);
}

bool SiteMap::apply(uint64_t epoch, const std::vector<CellDelta>& deltas, double stamp) {
  if (epoch != epoch_) {
    return false;
  }
  if (deltas.empty()) {
    return true;
  }
  ++version_;
  for (const CellDelta& d : deltas) {
    update(d.index, d.delta, stamp);
  }
  return true;
}

void SiteMap::integrate(const Vec3& origin, const std::vector<Vec3>& ends,
                        const std::vector<bool>& hits, double stamp) {
  ++version_;
  tracer_.trace(
      origin, ends, hits, res_, [](const Index3&) { return true; },
      [&](const Index3& i, int delta) { update(i, delta, stamp); });
}

Cell SiteMap::cell(const Index3& index) const {
  const auto it = tiles_.find(grid::tile_of(index));
  if (it == tiles_.end()) {
    return Cell::Unknown;
  }
  const int k = grid::offset_in_tile(index);
  return grid::cell_state(it->second.observed[k] != 0, it->second.log_odds[k]);
}

std::vector<CellDelta> SiteMap::observed_cells() const {
  std::vector<CellDelta> out;
  for (const auto& [key, tile] : tiles_) {
    const Index3 origin = key * kTileSize;
    for (int z = 0; z < kTileSize; ++z) {
      for (int y = 0; y < kTileSize; ++y) {
        for (int x = 0; x < kTileSize; ++x) {
          const Index3 index = origin + Index3(x, y, z);
          const int k = grid::offset_in_tile(index);
          if (tile.observed[k] != 0) {
            out.push_back({index, tile.log_odds[k]});
          }
        }
      }
    }
  }
  return out;
}

bool SiteMap::bounds(Index3& lo, Index3& hi) const {
  if (tiles_.empty()) {
    return false;
  }
  Index3 a = Index3::Constant(std::numeric_limits<int>::max());
  Index3 b = Index3::Constant(std::numeric_limits<int>::min());
  for (const auto& [key, tile] : tiles_) {
    a = a.cwiseMin(key);
    b = b.cwiseMax(key);
  }
  lo = a * kTileSize;
  hi = (b + Index3::Ones()) * kTileSize;
  return true;
}

PlanarView SiteMap::planar_view(double z_low_m, double z_high_m) const {
  Index3 lo, hi;
  if (!bounds(lo, hi)) {
    PlanarView empty;
    empty.resolution = res_;
    return empty;
  }
  return planar_view(z_low_m, z_high_m, lo, hi);
}

PlanarView SiteMap::planar_view(double z_low_m, double z_high_m, const Index3& lo,
                                const Index3& hi) const {
  PlanarView view;
  view.resolution = res_;
  view.origin = Index3(lo.x(), lo.y(), 0);
  view.nx = std::max(0, hi.x() - lo.x());
  view.ny = std::max(0, hi.y() - lo.y());
  view.cells.assign(static_cast<size_t>(view.nx) * view.ny, Cell::Unknown);
  const int z0 = grid::index_of(Vec3(0, 0, z_low_m), res_).z();
  const int z1 = grid::index_of(Vec3(0, 0, z_high_m), res_).z();
  if (z1 < z0 || view.cells.empty()) {
    return view;
  }
  // Walk tiles rather than cells: the map is sparse and most columns have no tile at all.
  // Combining with max works because Unknown < Free < Occupied.
  for (const auto& [key, tile] : tiles_) {
    const Index3 base = key * kTileSize;
    const int tz0 = std::max(z0, base.z());
    const int tz1 = std::min(z1, base.z() + kTileSize - 1);
    if (tz0 > tz1) {
      continue;
    }
    for (int lx = 0; lx < kTileSize; ++lx) {
      const int x = base.x() + lx - view.origin.x();
      if (x < 0 || x >= view.nx) {
        continue;
      }
      for (int ly = 0; ly < kTileSize; ++ly) {
        const int y = base.y() + ly - view.origin.y();
        if (y < 0 || y >= view.ny) {
          continue;
        }
        Cell& out = view.cells[static_cast<size_t>(x) * view.ny + y];
        for (int z = tz0; z <= tz1 && out != Cell::Occupied; ++z) {
          const int k = (lx * kTileSize + ly) * kTileSize + (z - base.z());
          out = std::max(out, grid::cell_state(tile.observed[k] != 0, tile.log_odds[k]));
        }
      }
    }
  }
  return view;
}

std::vector<Cell> SiteMap::coarse_view(int factor, const Index3& lo, const Index3& hi) const {
  if (factor < 1) {
    throw std::invalid_argument("coarse view factor must be at least 1");
  }
  const Index3 dims = (hi - lo).cwiseMax(0);
  std::vector<Cell> out(static_cast<size_t>(dims.prod()), Cell::Unknown);
  if (out.empty()) {
    return out;
  }
  for (const auto& [key, tile] : tiles_) {
    for (int k = 0; k < kTileCells; ++k) {
      if (!tile.observed[k]) {
        continue;
      }
      const Index3 fine = grid::cell_at(key, k);
      Index3 c;
      for (int a = 0; a < 3; ++a) {
        c[a] = grid::floor_div(fine[a], factor) - lo[a];
      }
      if ((c.array() < 0).any() || (c.array() >= dims.array()).any()) {
        continue;
      }
      Cell& cell = out[(static_cast<size_t>(c.x()) * dims.y() + c.y()) * dims.z() + c.z()];
      cell = std::max(cell, grid::cell_state(true, tile.log_odds[k]));
    }
  }
  return out;
}

std::vector<Index3> SiteMap::changed_since(uint64_t version) const {
  std::vector<Index3> out;
  for (const auto& [key, tile] : tiles_) {
    if (tile.version > version) {
      out.push_back(key);
    }
  }
  std::sort(out.begin(), out.end(), [](const Index3& a, const Index3& b) {
    return std::lexicographical_compare(a.data(), a.data() + 3, b.data(), b.data() + 3);
  });
  return out;
}

std::string SiteMap::encode_tiles(const std::vector<Index3>& keys) const {
  grid::Writer w;
  std::vector<const std::pair<const Index3, Tile>*> found;
  for (const Index3& key : keys) {
    const auto it = tiles_.find(key);
    if (it != tiles_.end()) {
      found.push_back(&*it);
    }
  }
  w.u32(static_cast<uint32_t>(found.size()));
  for (const auto* entry : found) {
    const Tile& tile = entry->second;
    w.i32(entry->first.x());
    w.i32(entry->first.y());
    w.i32(entry->first.z());
    w.u64(tile.version);
    uint64_t updated_bits = 0;
    std::memcpy(&updated_bits, &tile.updated, sizeof(updated_bits));
    w.u64(updated_bits);
    w.bytes(tile.log_odds.data(), tile.log_odds.size());
    for (int k = 0; k < kTileCells; k += 8) {
      uint8_t mask = 0;
      for (int b = 0; b < 8; ++b) {
        mask |= static_cast<uint8_t>(tile.observed[k + b] ? 1u << b : 0u);
      }
      w.u8(mask);
    }
  }
  return w.take();
}

std::string SiteMap::encode_all() const {
  std::vector<Index3> keys;
  keys.reserve(tiles_.size());
  for (const auto& [key, tile] : tiles_) {
    keys.push_back(key);
  }
  return encode_tiles(keys);
}

void SiteMap::load_tiles(const std::string& payload) {
  constexpr size_t kTileBytes = 12 + 8 + 8 + kTileCells + kTileCells / 8;
  grid::Reader r(payload);
  const uint32_t count = r.u32();
  if (count > payload.size() / kTileBytes) {
    throw std::invalid_argument("tiles: count larger than payload");
  }
  std::vector<std::pair<Index3, Tile>> loaded(count);
  uint64_t newest = version_;
  for (auto& [key, tile] : loaded) {
    key = r.index3();
    tile.version = r.u64();
    const uint64_t updated_bits = r.u64();
    std::memcpy(&tile.updated, &updated_bits, sizeof(updated_bits));
    r.bytes(tile.log_odds.data(), tile.log_odds.size());
    for (int8_t v : tile.log_odds) {
      if (v < grid::kMinLogOdds || v > grid::kMaxLogOdds) {
        throw std::invalid_argument("tiles: log-odds out of range");
      }
    }
    for (int k = 0; k < kTileCells; k += 8) {
      const uint8_t mask = r.u8();
      for (int b = 0; b < 8; ++b) {
        tile.observed[k + b] = (mask >> b) & 1u;
      }
    }
    newest = std::max(newest, tile.version);
  }
  if (!r.done()) {
    throw std::invalid_argument("tiles: trailing bytes");
  }
  for (auto& [key, tile] : loaded) {
    tiles_[key] = tile;
  }
  version_ = newest;
}

}  // namespace argus::sitemap

#include "argus/grid/evidence.hpp"

#include <algorithm>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <tuple>

#include "argus/grid/wire.hpp"

namespace argus::grid {

std::string encode_evidence(const std::vector<CellDelta>& deltas) {
  // Ordered so equal input gives equal bytes.
  std::map<std::tuple<int, int, int>, std::map<int, int>> tiles;
  for (const CellDelta& d : deltas) {
    const Index3 t = tile_of(d.index);
    tiles[{t.x(), t.y(), t.z()}][offset_in_tile(d.index)] += d.delta;
  }
  Writer w;
  w.u32(static_cast<uint32_t>(tiles.size()));
  for (const auto& [key, cells] : tiles) {
    w.i32(std::get<0>(key));
    w.i32(std::get<1>(key));
    w.i32(std::get<2>(key));
    w.u16(static_cast<uint16_t>(cells.size()));
    for (const auto& [offset, sum] : cells) {
      w.u16(static_cast<uint16_t>(offset));
      w.i8(static_cast<int8_t>(std::clamp(sum, -kDeltaLimit, kDeltaLimit)));
    }
  }
  return w.take();
}

std::vector<CellDelta> decode_evidence(const std::string& payload) {
  Reader r(payload);
  const uint32_t tile_count = r.u32();
  // Every tile needs at least its 14-byte header, which bounds a bogus count early.
  if (tile_count > payload.size() / 14) {
    throw std::invalid_argument("evidence: tile count larger than payload");
  }
  std::vector<CellDelta> out;
  for (uint32_t t = 0; t < tile_count; ++t) {
    const Index3 tile = r.index3();
    const uint16_t cells = r.u16();
    if (cells > kTileCells) {
      throw std::invalid_argument("evidence: more cells than a tile holds");
    }
    for (uint16_t c = 0; c < cells; ++c) {
      const uint16_t offset = r.u16();
      const int delta = r.i8();
      if (offset >= kTileCells) {
        throw std::invalid_argument("evidence: cell offset outside its tile");
      }
      if (delta < -kDeltaLimit || delta > kDeltaLimit) {
        throw std::invalid_argument("evidence: delta out of range");
      }
      out.push_back({cell_at(tile, offset), delta});
    }
  }
  if (!r.done()) {
    throw std::invalid_argument("evidence: trailing bytes");
  }
  return out;
}

}  // namespace argus::grid

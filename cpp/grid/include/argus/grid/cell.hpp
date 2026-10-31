#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Eigen/Core>

namespace argus::grid {

using Vec3 = Eigen::Vector3d;
using Index3 = Eigen::Vector3i;

enum class Cell : uint8_t { Unknown, Free, Occupied };

// Log-odds evidence per cell. Argo's local map and the site map share these, so evidence
// recorded on the robot means the same thing on the hub.
constexpr int kHit = 6;
constexpr int kMiss = -2;
constexpr int kMinLogOdds = -20;
constexpr int kMaxLogOdds = 12;

inline int8_t add_log_odds(int8_t value, int delta) {
  return static_cast<int8_t>(std::clamp(value + delta, kMinLogOdds, kMaxLogOdds));
}

inline Cell cell_state(bool observed, int8_t log_odds) {
  if (!observed) {
    return Cell::Unknown;
  }
  return log_odds > 0 ? Cell::Occupied : Cell::Free;
}

inline Index3 index_of(const Vec3& p, double res) {
  return Index3(static_cast<int>(std::floor(p.x() / res)), static_cast<int>(std::floor(p.y() / res)),
                static_cast<int>(std::floor(p.z() / res)));
}

inline Vec3 center_of(const Index3& i, double res) {
  return (i.cast<double>() + Vec3::Constant(0.5)) * res;
}

// Cells are grouped in cubic tiles for storage and transfer.
constexpr int kTileSize = 16;
constexpr int kTileCells = kTileSize * kTileSize * kTileSize;

inline int floor_div(int a, int b) {
  const int q = a / b;
  return (a % b != 0 && (a < 0) != (b < 0)) ? q - 1 : q;
}

inline Index3 tile_of(const Index3& cell) {
  return Index3(floor_div(cell.x(), kTileSize), floor_div(cell.y(), kTileSize),
                floor_div(cell.z(), kTileSize));
}

// Position of a cell inside its tile, 0 to kTileCells - 1.
inline int offset_in_tile(const Index3& cell) {
  const Index3 local = cell - tile_of(cell) * kTileSize;
  return (local.x() * kTileSize + local.y()) * kTileSize + local.z();
}

inline Index3 cell_at(const Index3& tile, int offset) {
  const Index3 local(offset / (kTileSize * kTileSize), (offset / kTileSize) % kTileSize,
                     offset % kTileSize);
  return tile * kTileSize + local;
}

}  // namespace argus::grid

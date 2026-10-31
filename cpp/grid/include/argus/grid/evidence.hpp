#pragma once

#include <string>
#include <vector>

#include "argus/grid/cell.hpp"

namespace argus::grid {

// Log-odds change for one cell, summed over the evidence collected since the last message.
// A delta of zero still means the cell was observed.
struct CellDelta {
  Index3 index = Index3::Zero();
  int delta = 0;
};

// Payload of LocalEvidence with EVIDENCE_ENCODING_VOXEL_DELTA, little-endian:
//
//   u32 tile_count
//   per tile: i32 tile_x, i32 tile_y, i32 tile_z, u16 cell_count,
//             per cell: u16 offset_in_tile, i8 delta
//
// Deltas for the same cell are summed. Sums are clipped to +-kDeltaLimit, which loses nothing
// because the log-odds range is narrower than that.
constexpr int kDeltaLimit = kMaxLogOdds - kMinLogOdds;

std::string encode_evidence(const std::vector<CellDelta>& deltas);

// Throws std::invalid_argument on a truncated or malformed payload.
std::vector<CellDelta> decode_evidence(const std::string& payload);

}  // namespace argus::grid

#pragma once

#include <cstdint>
#include <vector>

#include "argus/grid/evidence.hpp"
#include "argus/grid/trace.hpp"
#include "argus/node/types.hpp"

namespace argus::node {

using grid::Cell;
using grid::CellDelta;
using grid::Index3;

struct LocalMapConfig {
  double resolution_m = 0.1;
  double half_width_m = 8.0;
  // Vertical window relative to the base origin.
  double z_below_m = 1.0;
  double z_above_m = 1.0;
  // Keep the log-odds changes since the last take_evidence() call, for the hub's site map.
  // Only changes actually made are kept, plus cells observed for the first time.
  bool record_evidence = false;
};

// Short-horizon voxel map in the odom frame that moves with the robot. A ring buffer: cells
// that fall out of the window are forgotten, the rest keep their state when it moves.
class LocalMap {
 public:
  explicit LocalMap(const LocalMapConfig& config);

  // Moves the window so `base_position` sits in the middle horizontally.
  void recenter(const Vec3& base_position);

  // Integrates one sensor frame. Each ray clears the cells it passes through; rays that hit
  // mark their end cell occupied. Cells hit in this frame are not cleared by its other rays.
  void integrate(const Vec3& origin, const std::vector<Vec3>& ends, const std::vector<bool>& hits);

  Cell cell(const Index3& index) const;
  // Whether some sensor looked at this cell (see view.hpp), whatever its state. Only kept
  // here; it is not evidence and never reaches the hub.
  bool viewed(const Index3& index) const;
  void mark_viewed(const Index3& index);
  Cell at(const Vec3& point) const { return cell(index_of(point)); }
  bool contains(const Index3& index) const;

  Index3 index_of(const Vec3& point) const;
  Vec3 center_of(const Index3& index) const;

  const Index3& window_min() const { return min_; }
  const Index3& dims() const { return dims_; }
  double resolution() const { return config_.resolution_m; }
  const LocalMapConfig& config() const { return config_; }

  // Evidence recorded since the last call, in absolute cell indices. Empty unless
  // `record_evidence` is set. Cells that leave the window keep their recorded evidence.
  std::vector<CellDelta> take_evidence();

 private:
  size_t slot(const Index3& index) const;
  void clear_slab(int axis, int ring_coordinate);
  void update(const Index3& index, int delta);
  Index3 index_of_slot(size_t slot) const;
  void flush_pending();

  LocalMapConfig config_;
  Index3 dims_;
  Index3 min_;
  bool placed_ = false;
  std::vector<int8_t> log_odds_;
  std::vector<uint8_t> observed_;
  std::vector<uint8_t> viewed_;
  grid::FrameTracer tracer_;
  std::vector<int16_t> pending_;
  // Per slot: 0 not touched since the last flush, else whether it was observed before.
  static constexpr uint8_t kSeenBefore = 1;
  static constexpr uint8_t kNewlyObserved = 2;
  std::vector<uint8_t> pending_set_;
  std::vector<size_t> pending_slots_;
  std::vector<CellDelta> evidence_;
};

}  // namespace argus::node

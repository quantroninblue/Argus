#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>

#include "argus/grid/cell.hpp"

namespace argus::grid {

// Turns one range frame into per-cell evidence. Each ray clears the cells it passes through;
// rays that hit mark their end cell. Cells hit anywhere in the frame are not cleared by its
// other rays, so a ray grazing past an obstacle cannot erase it.
//
// `inside(index)` limits the walk to a map's extent: a ray stops once it leaves after having
// entered. `emit(index, delta)` receives kMiss or kHit per cell.
class FrameTracer {
 public:
  template <class Inside, class Emit>
  void trace(const Vec3& origin, const std::vector<Vec3>& ends, const std::vector<bool>& hits,
             double res, Inside inside, Emit emit) {
    hit_keys_.clear();
    for (size_t k = 0; k < ends.size(); ++k) {
      if (hits[k]) {
        const Index3 i = index_of(ends[k], res);
        if (inside(i)) {
          hit_keys_.push_back(key(i));
        }
      }
    }
    std::sort(hit_keys_.begin(), hit_keys_.end());
    hit_keys_.erase(std::unique(hit_keys_.begin(), hit_keys_.end()), hit_keys_.end());
    auto hit_this_frame = [&](const Index3& i) {
      return std::binary_search(hit_keys_.begin(), hit_keys_.end(), key(i));
    };

    const double inf = std::numeric_limits<double>::infinity();
    for (size_t k = 0; k < ends.size(); ++k) {
      const Vec3 d = ends[k] - origin;
      if (d.squaredNorm() < 1e-12) {
        continue;
      }
      Index3 cur = index_of(origin, res);
      const Index3 last = index_of(ends[k], res);
      Index3 step;
      Vec3 t_max;
      Vec3 t_delta;
      for (int a = 0; a < 3; ++a) {
        step[a] = d[a] > 0 ? 1 : (d[a] < 0 ? -1 : 0);
        if (step[a] == 0) {
          t_max[a] = inf;
          t_delta[a] = inf;
        } else {
          const double boundary = (cur[a] + (step[a] > 0 ? 1 : 0)) * res;
          t_max[a] = (boundary - origin[a]) / d[a];
          t_delta[a] = res / std::abs(d[a]);
        }
      }
      // Walk the cells from the sensor to the end point (Amanatides and Woo). The number of
      // crossings per axis comes from the end cell itself, so rounding in t_max can change the
      // order of crossings but never stop the walk short of the end cell. Ends that lie on a
      // cell boundary (wall faces on grid lines) otherwise lose their hit.
      Index3 remaining = (last - cur).cwiseAbs();
      for (int a = 0; a < 3; ++a) {
        if (remaining[a] == 0) {
          t_max[a] = inf;
        } else if (step[a] == 0 || (last[a] - cur[a]) * step[a] < 0) {
          // Rounding put the end cell behind the origin on this axis; skip this ray.
          remaining = Index3::Constant(-1);
          break;
        }
      }
      if (remaining.x() < 0) {
        continue;
      }
      bool entered = false;
      while (cur != last) {
        if (inside(cur)) {
          entered = true;
          if (!hit_this_frame(cur)) {
            emit(cur, kMiss);
          }
        } else if (entered) {
          break;
        }
        int a = 0;
        if (t_max[1] < t_max[a]) a = 1;
        if (t_max[2] < t_max[a]) a = 2;
        cur[a] += step[a];
        if (--remaining[a] == 0) {
          t_max[a] = inf;
        } else {
          t_max[a] += t_delta[a];
        }
      }
      if (cur == last && inside(last)) {
        if (hits[k]) {
          emit(last, kHit);
        } else if (!hit_this_frame(last)) {
          emit(last, kMiss);
        }
      }
    }
  }

 private:
  // 21 bits per axis covers +-100 km at 0.1 m.
  static uint64_t key(const Index3& i) {
    constexpr int64_t kOffset = 1 << 20;
    constexpr uint64_t kMask = (1u << 21) - 1;
    return ((static_cast<uint64_t>(i.x() + kOffset) & kMask) << 42) |
           ((static_cast<uint64_t>(i.y() + kOffset) & kMask) << 21) |
           (static_cast<uint64_t>(i.z() + kOffset) & kMask);
  }

  std::vector<uint64_t> hit_keys_;
};

}  // namespace argus::grid

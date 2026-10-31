#include "argus/node/local_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace argus::node {

namespace {

constexpr int kHit = 6;
constexpr int kMiss = -2;
constexpr int kMinLogOdds = -20;
constexpr int kMaxLogOdds = 12;

int ring(int i, int n) {
  const int r = i % n;
  return r < 0 ? r + n : r;
}

}  // namespace

LocalMap::LocalMap(const LocalMapConfig& config) : config_(config) {
  const double res = config_.resolution_m;
  const int horizontal = 2 * static_cast<int>(std::ceil(config_.half_width_m / res));
  const int vertical =
      std::max(1, static_cast<int>(std::ceil((config_.z_below_m + config_.z_above_m) / res)));
  dims_ = Index3(horizontal, horizontal, vertical);
  min_ = Index3::Zero();
  const size_t n = static_cast<size_t>(dims_.prod());
  log_odds_.assign(n, 0);
  observed_.assign(n, 0);
}

Index3 LocalMap::index_of(const Vec3& p) const {
  const double res = config_.resolution_m;
  return Index3(static_cast<int>(std::floor(p.x() / res)), static_cast<int>(std::floor(p.y() / res)),
                static_cast<int>(std::floor(p.z() / res)));
}

Vec3 LocalMap::center_of(const Index3& i) const {
  return (i.cast<double>() + Vec3::Constant(0.5)) * config_.resolution_m;
}

bool LocalMap::contains(const Index3& i) const {
  return (i.array() >= min_.array()).all() && (i.array() < (min_ + dims_).array()).all();
}

size_t LocalMap::slot(const Index3& i) const {
  const int x = ring(i.x(), dims_.x());
  const int y = ring(i.y(), dims_.y());
  const int z = ring(i.z(), dims_.z());
  return (static_cast<size_t>(x) * dims_.y() + y) * dims_.z() + z;
}

Cell LocalMap::cell(const Index3& i) const {
  if (!contains(i)) {
    return Cell::Unknown;
  }
  const size_t s = slot(i);
  if (!observed_[s]) {
    return Cell::Unknown;
  }
  return log_odds_[s] > 0 ? Cell::Occupied : Cell::Free;
}

void LocalMap::clear_slab(int axis, int r) {
  Index3 i;
  i[axis] = r;
  const int a = (axis + 1) % 3;
  const int b = (axis + 2) % 3;
  for (i[a] = 0; i[a] < dims_[a]; ++i[a]) {
    for (i[b] = 0; i[b] < dims_[b]; ++i[b]) {
      const size_t s = (static_cast<size_t>(i.x()) * dims_.y() + i.y()) * dims_.z() + i.z();
      log_odds_[s] = 0;
      observed_[s] = 0;
    }
  }
}

void LocalMap::recenter(const Vec3& base) {
  const Index3 c = index_of(base);
  const Index3 next(c.x() - dims_.x() / 2, c.y() - dims_.y() / 2,
                    static_cast<int>(std::floor((base.z() - config_.z_below_m) / config_.resolution_m)));
  if (!placed_) {
    min_ = next;
    placed_ = true;
    return;
  }
  if (next == min_) {
    return;
  }
  for (int axis = 0; axis < 3; ++axis) {
    const int lo = min_[axis];
    const int hi = min_[axis] + dims_[axis];
    if (std::abs(next[axis] - lo) >= dims_[axis]) {
      std::fill(log_odds_.begin(), log_odds_.end(), 0);
      std::fill(observed_.begin(), observed_.end(), 0);
      break;
    }
    for (int j = lo; j < hi; ++j) {
      if (j < next[axis] || j >= next[axis] + dims_[axis]) {
        clear_slab(axis, ring(j, dims_[axis]));
      }
    }
  }
  min_ = next;
}

void LocalMap::update(const Index3& i, int delta) {
  const size_t s = slot(i);
  log_odds_[s] = static_cast<int8_t>(std::clamp(log_odds_[s] + delta, kMinLogOdds, kMaxLogOdds));
  observed_[s] = 1;
}

void LocalMap::integrate(const Vec3& origin, const std::vector<Vec3>& ends,
                         const std::vector<bool>& hits) {
  frame_hits_.clear();
  for (size_t k = 0; k < ends.size(); ++k) {
    if (hits[k]) {
      const Index3 i = index_of(ends[k]);
      if (contains(i)) {
        frame_hits_.push_back(slot(i));
      }
    }
  }
  std::sort(frame_hits_.begin(), frame_hits_.end());
  frame_hits_.erase(std::unique(frame_hits_.begin(), frame_hits_.end()), frame_hits_.end());
  auto hit_this_frame = [&](const Index3& i) {
    return std::binary_search(frame_hits_.begin(), frame_hits_.end(), slot(i));
  };

  const double res = config_.resolution_m;
  const double inf = std::numeric_limits<double>::infinity();
  for (size_t k = 0; k < ends.size(); ++k) {
    const Vec3 d = ends[k] - origin;
    if (d.squaredNorm() < 1e-12) {
      continue;
    }
    Index3 cur = index_of(origin);
    const Index3 last = index_of(ends[k]);
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
    // Walk the cells from the sensor to the end point (Amanatides and Woo).
    const int max_steps = (last - cur).cwiseAbs().sum() + 3;
    bool entered = false;
    for (int n = 0; n < max_steps && cur != last; ++n) {
      if (contains(cur)) {
        entered = true;
        if (!hit_this_frame(cur)) {
          update(cur, kMiss);
        }
      } else if (entered) {
        break;
      }
      int a = 0;
      if (t_max[1] < t_max[a]) a = 1;
      if (t_max[2] < t_max[a]) a = 2;
      if (t_max[a] > 1.0) {
        break;
      }
      cur[a] += step[a];
      t_max[a] += t_delta[a];
    }
    if (cur == last && contains(last)) {
      if (hits[k]) {
        update(last, kHit);
      } else if (!hit_this_frame(last)) {
        update(last, kMiss);
      }
    }
  }
}

}  // namespace argus::node

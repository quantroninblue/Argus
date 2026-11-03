#include "argus/node/local_map.hpp"

#include <algorithm>
#include <cmath>

namespace argus::node {

namespace {

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
  viewed_.assign(n, 0);
  free_frames_.assign(n, 0);
  last_free_frame_.assign(n, 0);
  ever_free_.assign(n, 0);
  drop_.assign(n, 0);
  if (config_.record_evidence) {
    pending_.assign(n, 0);
    pending_set_.assign(n, 0);
  }
}

Index3 LocalMap::index_of(const Vec3& p) const { return grid::index_of(p, config_.resolution_m); }

Vec3 LocalMap::center_of(const Index3& i) const { return grid::center_of(i, config_.resolution_m); }

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
  if (drop_[s]) {
    return Cell::Occupied;
  }
  return grid::cell_state(observed_[s] != 0, log_odds_[s]);
}

int LocalMap::log_odds(const Index3& i) const {
  if (!contains(i)) {
    return 0;
  }
  const size_t s = slot(i);
  return observed_[s] ? log_odds_[s] : 0;
}

bool LocalMap::viewed(const Index3& i) const { return contains(i) && viewed_[slot(i)] != 0; }

void LocalMap::mark_viewed(const Index3& i) {
  if (contains(i)) {
    viewed_[slot(i)] = 1;
  }
}

bool LocalMap::drop(const Index3& i) const { return contains(i) && drop_[slot(i)] != 0; }

void LocalMap::mark_drop(const Index3& i) {
  if (contains(i)) {
    drop_[slot(i)] = 1;
  }
}

void LocalMap::clear_drop_column(const Index3& i) {
  if (!contains(i)) {
    return;
  }
  Index3 c = i;
  for (c.z() = min_.z(); c.z() < min_.z() + dims_.z(); ++c.z()) {
    drop_[slot(c)] = 0;
  }
}

void LocalMap::forget(const Index3& i) {
  if (!contains(i)) {
    return;
  }
  const size_t s = slot(i);
  log_odds_[s] = 0;
  observed_[s] = 0;
  viewed_[s] = 0;
  free_frames_[s] = 0;
  ever_free_[s] = 0;
}

namespace {

// The 18-neighbourhood: faces and edges of the cube. `f` gets the neighbour and its dz.
template <typename F>
void for_each_neighbour(const Index3& i, F&& f) {
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        const int n = std::abs(dx) + std::abs(dy) + std::abs(dz);
        if (n == 1 || n == 2) {
          f(Index3(i.x() + dx, i.y() + dy, i.z() + dz), dz);
        }
      }
    }
  }
}

}  // namespace

bool LocalMap::ever_free(const Index3& i) const { return contains(i) && ever_free_[slot(i)] != 0; }

void LocalMap::try_ever_free(const Index3& i, size_t s) {
  // Dynablox asks for every neighbour observed and not occupied. A single scan plane never
  // observes the cells above and below it, so an unobserved neighbour above or below no
  // sensor can see does not count (set_vertically_unseen). One inside a box, under its top
  // edge, still does.
  bool clear = true;
  for_each_neighbour(i, [&](const Index3& n, int dz) {
    if (!clear) {
      return;
    }
    const Cell c = contains(n) ? cell(n) : Cell::Unknown;
    if (c == Cell::Occupied ||
        (c == Cell::Unknown && (dz == 0 || !contains(n) || !vertically_unseen_ || !vertically_unseen_(n)))) {
      clear = false;
    }
  });
  ever_free_[s] = clear ? 1 : 0;
}

void LocalMap::revoke_ever_free(const Index3& i) {
  if (contains(i)) {
    ever_free_[slot(i)] = 0;
  }
  for_each_neighbour(i, [&](const Index3& n, int) {
    if (contains(n)) {
      ever_free_[slot(n)] = 0;
    }
  });
}

Index3 LocalMap::index_of_slot(size_t s) const {
  const int z = static_cast<int>(s % dims_.z());
  const int y = static_cast<int>((s / dims_.z()) % dims_.y());
  const int x = static_cast<int>(s / (static_cast<size_t>(dims_.z()) * dims_.y()));
  const Index3 r(x, y, z);
  Index3 out;
  for (int a = 0; a < 3; ++a) {
    out[a] = min_[a] + ring(r[a] - min_[a], dims_[a]);
  }
  return out;
}

void LocalMap::flush_pending() {
  for (size_t s : pending_slots_) {
    // A cell that was already observed and did not change tells the hub nothing new.
    if (pending_[s] != 0 || pending_set_[s] == kNewlyObserved) {
      evidence_.push_back({index_of_slot(s), pending_[s]});
    }
    pending_[s] = 0;
    pending_set_[s] = 0;
  }
  pending_slots_.clear();
}

std::vector<CellDelta> LocalMap::take_evidence() {
  flush_pending();
  return std::move(evidence_);
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
      viewed_[s] = 0;
      free_frames_[s] = 0;
      last_free_frame_[s] = 0;
      ever_free_[s] = 0;
      drop_[s] = 0;
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
  // Slots are about to be reused for other cells; settle what they recorded first.
  flush_pending();
  for (int axis = 0; axis < 3; ++axis) {
    const int lo = min_[axis];
    const int hi = min_[axis] + dims_[axis];
    if (std::abs(next[axis] - lo) >= dims_[axis]) {
      std::fill(log_odds_.begin(), log_odds_.end(), 0);
      std::fill(observed_.begin(), observed_.end(), 0);
      std::fill(viewed_.begin(), viewed_.end(), 0);
      std::fill(drop_.begin(), drop_.end(), 0);
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
  const int8_t before = log_odds_[s];
  log_odds_[s] = grid::add_log_odds(before, delta);
  if (delta < 0) {
    // Seen free in this frame: count it once per frame, and see whether it is now ever-free.
    if (last_free_frame_[s] != frame_) {
      last_free_frame_[s] = frame_;
      if (free_frames_[s] < 255) {
        ++free_frames_[s];
      }
      if (!ever_free_[s] && free_frames_[s] >= config_.ever_free_burn_in) {
        try_ever_free(i, s);
      }
    }
  } else if (delta > 0) {
    // Occupied now: it starts its burn-in again. Its neighbours keep ever-free, as in
    // Dynablox, where only a voxel occupied for many frames takes it from them (Argo does that
    // in Node, static_after_s); revoking them on every hit let a walker's first returns keep
    // the space just ahead of it from ever counting as free.
    free_frames_[s] = 0;
    ever_free_[s] = 0;
  }
  if (config_.record_evidence) {
    // Record the change actually made, so the hub's cell follows this one step for step.
    pending_[s] = static_cast<int16_t>(pending_[s] + log_odds_[s] - before);
    if (!pending_set_[s]) {
      pending_set_[s] = observed_[s] ? kSeenBefore : kNewlyObserved;
      pending_slots_.push_back(s);
    }
  }
  observed_[s] = 1;
}

void LocalMap::integrate(const Vec3& origin, const std::vector<Vec3>& ends,
                         const std::vector<bool>& hits) {
  if (++frame_ == 0) {
    frame_ = 1;  // 0 is "never counted"
  }
  tracer_.trace(
      origin, ends, hits, config_.resolution_m, [this](const Index3& i) { return contains(i); },
      [this](const Index3& i, int delta) { update(i, delta); });
}

}  // namespace argus::node

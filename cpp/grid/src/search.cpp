#include "argus/grid/search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace argus::grid {

namespace {

constexpr double kBig = 1e9;

// One pass of the Felzenszwalb-Huttenlocher squared distance transform.
void transform_line(std::vector<double>& f, std::vector<double>& d, std::vector<int>& v,
                    std::vector<double>& z, int n) {
  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<double>::infinity();
  z[1] = std::numeric_limits<double>::infinity();
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

}  // namespace

std::vector<float> distance_field(const std::vector<uint8_t>& occupied, const Index3& dims, double res) {
  const size_t n = occupied.size();
  std::vector<double> sq(n);
  for (size_t i = 0; i < n; ++i) {
    sq[i] = occupied[i] ? 0.0 : kBig;
  }
  const int longest = dims.maxCoeff();
  std::vector<double> f(longest), d(longest), z(longest + 1);
  std::vector<int> v(longest);
  const Index3 stride(dims.y() * dims.z(), dims.z(), 1);
  for (int axis = 0; axis < 3; ++axis) {
    const int len = dims[axis];
    if (len < 2) {
      continue;
    }
    const int a = (axis + 1) % 3;
    const int b = (axis + 2) % 3;
    for (int i = 0; i < dims[a]; ++i) {
      for (int j = 0; j < dims[b]; ++j) {
        const size_t base = static_cast<size_t>(i) * stride[a] + static_cast<size_t>(j) * stride[b];
        for (int q = 0; q < len; ++q) {
          f[q] = sq[base + static_cast<size_t>(q) * stride[axis]];
        }
        transform_line(f, d, v, z, len);
        for (int q = 0; q < len; ++q) {
          sq[base + static_cast<size_t>(q) * stride[axis]] = d[q];
        }
      }
    }
  }
  std::vector<float> out(n);
  for (size_t i = 0; i < n; ++i) {
    out[i] = sq[i] >= kBig ? static_cast<float>(SearchGrid::kFar)
                           : static_cast<float>(std::sqrt(sq[i]) * res);
  }
  return out;
}

void SearchGrid::assign(const Index3& origin, const Index3& dims, double res, bool planar, double z,
                        std::vector<Cell> cells) {
  planar_ = planar;
  origin_ = origin;
  if (planar_) {
    origin_.z() = 0;
  }
  dims_ = dims;
  res_ = res;
  z_ = z;
  cells_ = std::move(cells);
  std::vector<uint8_t> occupied(cells_.size());
  for (size_t k = 0; k < cells_.size(); ++k) {
    occupied[k] = cells_[k] == Cell::Occupied;
  }
  clearance_ = distance_field(occupied, dims_, res_);
}

bool SearchGrid::contains(const Index3& i) const {
  return (i.array() >= 0).all() && (i.array() < dims_.array()).all();
}

Index3 SearchGrid::local_of(const Vec3& p) const {
  Index3 i(static_cast<int>(std::floor(p.x() / res_)) - origin_.x(),
           static_cast<int>(std::floor(p.y() / res_)) - origin_.y(), 0);
  if (!planar_) {
    i.z() = static_cast<int>(std::floor(p.z() / res_)) - origin_.z();
  }
  return i;
}

Vec3 SearchGrid::center_of(const Index3& i) const {
  Vec3 c = ((i + origin_).cast<double>() + Vec3::Constant(0.5)) * res_;
  if (planar_) {
    c.z() = z_;
  }
  return c;
}

Cell SearchGrid::state_at(const Vec3& p) const {
  const Index3 i = local_of(p);
  return contains(i) ? state(i) : Cell::Unknown;
}

double SearchGrid::clearance_at(const Vec3& p) const {
  const Index3 i = local_of(p);
  return contains(i) ? clearance(i) : kFar;
}

Vec3 SearchGrid::lower() const {
  Vec3 v = origin_.cast<double>() * res_;
  if (planar_) {
    v.z() = z_;
  }
  return v;
}

Vec3 SearchGrid::upper() const {
  Vec3 v = (origin_ + dims_).cast<double>() * res_;
  if (planar_) {
    v.z() = z_;
  }
  return v;
}

PathResult PathSearch::find(const SearchGrid& ws, const PathRequest& req) {
  PathResult result;
  const Index3 dims = ws.dims();
  const size_t n = static_cast<size_t>(dims.prod());
  if (g_.size() != n) {
    g_.assign(n, 0.0f);
    parent_.assign(n, -1);
    seen_.assign(n, 0);
    closed_.assign(n, 0);
    generation_ = 0;
  }
  if (++generation_ == 0) {
    std::fill(seen_.begin(), seen_.end(), 0);
    generation_ = 1;
  }

  const Index3 start = ws.local_of(req.start);
  if (!ws.contains(start)) {
    return result;
  }

  // Clip the goal to the window, keeping two cells of margin.
  Vec3 goal = req.goal;
  const Vec3 margin = Vec3::Constant(2.0 * ws.resolution());
  Vec3 lo = ws.lower() + margin;
  Vec3 hi = ws.upper() - margin;
  if (ws.planar()) {
    lo.z() = hi.z() = goal.z() = req.start.z();
  }
  const Vec3 d = goal - req.start;
  double t = 1.0;
  for (int a = 0; a < 3; ++a) {
    if (d[a] > 1e-12) t = std::min(t, (hi[a] - req.start[a]) / d[a]);
    if (d[a] < -1e-12) t = std::min(t, (lo[a] - req.start[a]) / d[a]);
  }
  goal = req.start + std::max(0.0, t) * d;
  Index3 goal_i = ws.local_of(goal);
  if (!ws.contains(goal_i)) {
    goal_i = start;
  }

  const double res = ws.resolution();
  auto heuristic = [&](const Index3& i) { return (i - goal_i).cast<double>().norm() * res; };
  auto mark = [&](size_t k, float g, int parent) {
    seen_[k] = generation_;
    closed_[k] = 0;
    g_[k] = g;
    parent_[k] = parent;
  };

  using Entry = std::pair<double, size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  const size_t s = ws.linear(start);
  mark(s, 0.0f, -1);
  open.emplace(heuristic(start), s);
  size_t best = s;
  double best_h = heuristic(start);
  size_t goal_k = ws.linear(goal_i);
  const bool bounded = req.corridor.size() >= 2 && req.corridor_m > 0.0;
  const int dz_max = ws.planar() ? 0 : 1;
  int expansions = 0;

  auto to_index = [&](size_t k) {
    const int z = static_cast<int>(k % dims.z());
    const int y = static_cast<int>((k / dims.z()) % dims.y());
    const int x = static_cast<int>(k / (static_cast<size_t>(dims.z()) * dims.y()));
    return Index3(x, y, z);
  };

  while (!open.empty() && expansions < req.max_expansions) {
    const auto [f, k] = open.top();
    open.pop();
    if (closed_[k]) {
      continue;
    }
    closed_[k] = 1;
    ++expansions;
    const Index3 cur = to_index(k);
    const double h = heuristic(cur);
    if (h < best_h) {
      best_h = h;
      best = k;
    }
    if (k == goal_k) {
      result.reached = true;
      best = k;
      break;
    }
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -dz_max; dz <= dz_max; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const Index3 nb = cur + Index3(dx, dy, dz);
          if (!ws.contains(nb)) {
            continue;
          }
          const double clear = ws.clearance(nb);
          if (clear < req.min_clearance_m) {
            continue;
          }
          if (bounded && distance_to_polyline(req.corridor, ws.center_of(nb), ws.planar()) > req.corridor_m) {
            continue;
          }
          const size_t nk = ws.linear(nb);
          if (seen_[nk] == generation_ && closed_[nk]) {
            continue;
          }
          double cost = 1.0;
          if (ws.state(nb) == Cell::Unknown) {
            cost += req.unknown_penalty;
          }
          if (req.comfort_clearance_m > 0.0 && clear < req.comfort_clearance_m) {
            cost += 2.0 * (req.comfort_clearance_m - clear) / req.comfort_clearance_m;
          }
          const double g = g_[k] + cost * std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) * res;
          if (seen_[nk] != generation_ || g < g_[nk]) {
            mark(nk, static_cast<float>(g), static_cast<int>(k));
            open.emplace(g + heuristic(nb), nk);
          }
        }
      }
    }
  }

  result.exhausted = !result.reached && open.empty();
  result.goal_blocked = ws.contains(goal_i) && goal_i != start &&
                        (ws.state(goal_i) == Cell::Occupied || ws.clearance(goal_i) < req.min_clearance_m);

  std::vector<Vec3> reversed;
  for (int k = static_cast<int>(best); k >= 0; k = parent_[k]) {
    reversed.push_back(ws.center_of(to_index(static_cast<size_t>(k))));
  }
  result.points.assign(reversed.rbegin(), reversed.rend());
  if (!result.points.empty()) {
    result.points.front() = req.start;
  }
  if (result.reached && goal_k != s) {
    result.points.push_back(goal);
  }
  return result;
}


double distance_to_polyline(const std::vector<Vec3>& line, const Vec3& p, bool planar) {
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    Vec3 a = line[i];
    Vec3 b = line[i + 1];
    Vec3 q = p;
    if (planar) {
      a.z() = b.z() = q.z() = 0.0;
    }
    const Vec3 ab = b - a;
    const double len2 = ab.squaredNorm();
    const double t = len2 > 1e-12 ? std::clamp((q - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    best = std::min(best, (a + t * ab - q).norm());
  }
  return best;
}

}  // namespace argus::grid

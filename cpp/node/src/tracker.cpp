#include "argus/node/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace argus::node {

namespace {

constexpr double kMinRadius = 0.1;
constexpr size_t kTrailLength = 40;

}  // namespace

ObstacleTracker::ObstacleTracker(TrackerConfig config) : config_(config) {}

std::vector<ObstacleTracker::Detection> ObstacleTracker::cluster(const std::vector<Vec3>& returns) const {
  // Single-linkage clusters on the floor plane: two returns closer than the gap are one
  // thing. Frames hold at most a few hundred such returns, so the quadratic pass is cheap.
  const size_t n = returns.size();
  std::vector<int> label(n, -1);
  std::vector<Detection> out;
  const double gap2 = config_.cluster_gap_m * config_.cluster_gap_m;
  int next = 0;
  std::vector<size_t> stack;
  for (size_t i = 0; i < n; ++i) {
    if (label[i] >= 0) {
      continue;
    }
    label[i] = next;
    stack.assign(1, i);
    Eigen::Vector2d lo = returns[i].head<2>();
    Eigen::Vector2d hi = lo;
    double z_low = returns[i].z();
    double z_high = z_low;
    int count = 0;
    while (!stack.empty()) {
      const size_t a = stack.back();
      stack.pop_back();
      ++count;
      lo = lo.cwiseMin(returns[a].head<2>());
      hi = hi.cwiseMax(returns[a].head<2>());
      z_low = std::min(z_low, returns[a].z());
      z_high = std::max(z_high, returns[a].z());
      for (size_t b = 0; b < n; ++b) {
        if (label[b] < 0 && (returns[a].head<2>() - returns[b].head<2>()).squaredNorm() <= gap2) {
          label[b] = next;
          stack.push_back(b);
        }
      }
    }
    ++next;
    const Eigen::Vector2d extent = hi - lo;
    if (count < config_.min_cluster_hits || extent.maxCoeff() > config_.max_extent_m) {
      continue;
    }
    out.push_back({(lo + hi) / 2.0, std::max(kMinRadius, extent.norm() / 2.0), lo, hi, z_low, z_high});
  }
  return out;
}

void ObstacleTracker::predict(Track& t, double to) const {
  const double dt = to - t.stamp;
  if (dt <= 0.0) {
    return;
  }
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = F(1, 3) = dt;
  // Discrete white-noise acceleration.
  const double q = config_.accel_sigma * config_.accel_sigma;
  const double dt2 = dt * dt;
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  for (int a = 0; a < 2; ++a) {
    Q(a, a) = q * dt2 * dt2 / 4.0;
    Q(a, a + 2) = Q(a + 2, a) = q * dt2 * dt / 2.0;
    Q(a + 2, a + 2) = q * dt2;
  }
  t.x = F * t.x;
  t.P = F * t.P * F.transpose() + Q;
  t.stamp = to;
}

void ObstacleTracker::correct(Track& t, const Eigen::Vector2d& z) const {
  Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
  H(0, 0) = H(1, 1) = 1.0;
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * config_.measurement_sigma_m * config_.measurement_sigma_m;
  const Eigen::Matrix2d S = H * t.P * H.transpose() + R;
  const Eigen::Matrix<double, 4, 2> K = t.P * H.transpose() * S.inverse();
  t.x += K * (z - H * t.x);
  t.P = (Eigen::Matrix4d::Identity() - K * H) * t.P;
}

TrackedObstacle ObstacleTracker::predicted(const Track& t, double to) const {
  TrackedObstacle o = t.out;
  const double dt = std::max(0.0, to - t.stamp);
  o.velocity = t.x.tail<2>();
  o.position = t.x.head<2>() + o.velocity * dt;
  Track ahead = t;
  predict(ahead, to);
  o.position_sigma_m = std::sqrt(std::max(ahead.P(0, 0), ahead.P(1, 1)));
  o.velocity_sigma_mps = std::sqrt(std::max(ahead.P(2, 2), ahead.P(3, 3)));
  // Moving needs both speed and real travel: a cluster on structure drifts as more of it
  // comes into view, but it never gets anywhere.
  const double travelled = o.trail.empty() ? 0.0 : (t.x.head<2>() - o.trail.front()).norm();
  o.confirmed = confirmed(t);
  o.moving = o.confirmed && o.velocity.norm() > config_.moving_mps && travelled >= config_.min_travel_m &&
             edges_shifted(t);
  return o;
}

bool ObstacleTracker::edges_shifted(const Track& t) const {
  // Something coming into view (the rest of a wall, the top of a low box) grows at one edge
  // while the other stays put, so its centre drifts; something moving shifts both edges.
  if (t.boxes.size() < 2) {
    return false;
  }
  const auto& [first_lo, first_hi] = t.boxes.front();
  const auto& [lo, hi] = t.boxes.back();
  const Eigen::Vector2d d = (lo + hi) / 2.0 - (first_lo + first_hi) / 2.0;
  const double n = d.norm();
  if (n < 1e-9) {
    return false;
  }
  const Eigen::Vector2d u = d / n;
  // Projection of a box's near and far edge on u.
  const auto span = [&](const Eigen::Vector2d& lo, const Eigen::Vector2d& hi) {
    double a = std::numeric_limits<double>::max();
    double b = std::numeric_limits<double>::lowest();
    for (const Eigen::Vector2d& c : {lo, hi, Eigen::Vector2d(lo.x(), hi.y()), Eigen::Vector2d(hi.x(), lo.y())}) {
      a = std::min(a, u.dot(c));
      b = std::max(b, u.dot(c));
    }
    return std::make_pair(a, b);
  };
  const auto [a0, b0] = span(first_lo, first_hi);
  const auto [a1, b1] = span(lo, hi);
  return std::min(a1 - a0, b1 - b0) >= config_.min_edge_shift_m;
}

void ObstacleTracker::update(double stamp, const std::vector<Vec3>& returns) {
  // Drop what has gone unseen too long first, so a stale track cannot take a new detection.
  for (auto it = tracks_.begin(); it != tracks_.end();) {
    if (stamp - it->out.last_seen > config_.lifetime_s) {
      if (confirmed(*it)) {
        lost_.push_back(predicted(*it, stamp));
      }
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
  const std::vector<Detection> detections = cluster(returns);
  if (detections.empty()) {
    return;
  }
  // Greedy association, nearest pair first, inside the gate and below a believable speed.
  struct Pair {
    double d;
    size_t track;
    size_t detection;
  };
  std::vector<Pair> pairs;
  for (size_t ti = 0; ti < tracks_.size(); ++ti) {
    const TrackedObstacle p = predicted(tracks_[ti], stamp);
    const double dt = std::max(1e-3, stamp - tracks_[ti].out.last_seen);
    for (size_t di = 0; di < detections.size(); ++di) {
      const double d = (detections[di].centre - p.position).norm();
      const double jump = (detections[di].centre - tracks_[ti].x.head<2>()).norm() / dt;
      if (d <= config_.gate_m + detections[di].radius_m && jump <= config_.max_speed_mps + config_.gate_m / dt) {
        pairs.push_back({d, ti, di});
      }
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d < b.d; });
  std::vector<bool> track_used(tracks_.size(), false);
  std::vector<bool> detection_used(detections.size(), false);
  for (const Pair& p : pairs) {
    if (track_used[p.track] || detection_used[p.detection]) {
      continue;
    }
    track_used[p.track] = detection_used[p.detection] = true;
    Track& t = tracks_[p.track];
    predict(t, stamp);
    correct(t, detections[p.detection].centre);
    const double speed = t.x.tail<2>().norm();
    if (speed > config_.max_speed_mps) {
      t.x.tail<2>() *= config_.max_speed_mps / speed;
    }
    t.hits += 1;
    t.out.last_seen = stamp;
    t.out.radius_m = std::max(t.out.radius_m * 0.8, detections[p.detection].radius_m);
    t.boxes.emplace_back(detections[p.detection].lo, detections[p.detection].hi);
    t.out.z_low_m = detections[p.detection].z_low;
    t.out.z_high_m = detections[p.detection].z_high;
    t.out.trail.push_back(detections[p.detection].centre);
    if (t.out.trail.size() > kTrailLength) {
      t.out.trail.erase(t.out.trail.begin());
      t.boxes.erase(t.boxes.begin());
    }
  }
  for (size_t di = 0; di < detections.size(); ++di) {
    if (detection_used[di]) {
      continue;
    }
    Track t;
    t.out.id = next_id_++;
    t.out.first_seen = t.out.last_seen = stamp;
    t.out.radius_m = detections[di].radius_m;
    t.out.trail.push_back(detections[di].centre);
    t.boxes.emplace_back(detections[di].lo, detections[di].hi);
    t.out.z_low_m = detections[di].z_low;
    t.out.z_high_m = detections[di].z_high;
    t.x.head<2>() = detections[di].centre;
    t.P.diagonal() << 0.05, 0.05, 4.0, 4.0;  // position close to the detection; velocity unknown
    t.stamp = stamp;
    t.hits = 1;
    tracks_.push_back(std::move(t));
  }
}

std::vector<TrackedObstacle> ObstacleTracker::tracks(double now, bool moving_only) const {
  std::vector<TrackedObstacle> out;
  for (const Track& t : tracks_) {
    if (!confirmed(t)) {
      continue;
    }
    TrackedObstacle o = predicted(t, now);
    if (!moving_only || o.moving) {
      out.push_back(std::move(o));
    }
  }
  return out;
}

bool ObstacleTracker::explains(const Vec3& p, double stamp) const {
  for (const Track& t : tracks_) {
    if (!confirmed(t)) {
      continue;
    }
    const TrackedObstacle o = predicted(t, stamp);
    if (o.moving && (p.head<2>() - o.position).norm() <= o.radius_m + config_.cluster_gap_m) {
      return true;
    }
  }
  return false;
}

std::vector<TrackedObstacle> ObstacleTracker::all(double now) const {
  std::vector<TrackedObstacle> out;
  for (const Track& t : tracks_) {
    out.push_back(predicted(t, now));
  }
  return out;
}

std::vector<TrackedObstacle> ObstacleTracker::take_lost() {
  std::vector<TrackedObstacle> out;
  out.swap(lost_);
  return out;
}

}  // namespace argus::node

#pragma once

#include <utility>
#include <vector>

#include <Eigen/Core>

#include "argus/node/types.hpp"

namespace argus::node {

// Things that move near the robot (people, carts), tracked on the floor plane from range
// returns that land in ever-free space (local_map.hpp; Dynablox, Schmid et al. 2023).
//
// Pattern from Autoware's multi_object_tracker: detections each frame, associated to tracks
// inside a gate, one constant-velocity Kalman filter per track, a track dropped after a
// second unseen. Detections here are Euclidean clusters of those returns (Argo has range
// sensors, not classified objects), and association is greedy nearest-first, which is enough
// for the handful of things near one robot.
struct TrackerConfig {
  double cluster_gap_m = 0.3;   // returns closer than this belong to one detection
  int min_cluster_hits = 2;
  double max_extent_m = 1.2;    // larger clusters are structure coming into view, not walkers
  double gate_m = 0.8;          // farthest a detection may land from a track's prediction
  double lifetime_s = 1.0;      // unseen this long, a track is dropped
  int confirm_hits = 3;         // frames with a detection before a track counts
  double moving_mps = 0.2;      // slower is still
  double min_travel_m = 0.3;    // and it counts as moving only once it has gone this far,
  double min_edge_shift_m = 0.15;  // with both its edges along the way shifted at least this much
  double accel_sigma = 3.0;     // how sharply things change velocity, m/s^2 (process noise)
  double measurement_sigma_m = 0.1;
  double max_speed_mps = 4.0;   // faster associations are taken as two different things
};

struct TrackedObstacle {
  int id = 0;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();  // odom, predicted to the time asked for
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  double radius_m = 0.0;  // half the cluster's extent, at least a cell
  double z_low_m = 0.0;   // height span of its returns, odom
  double z_high_m = 0.0;
  // Set by Node: moving now, or moved lately. Things like this are kept at a distance and
  // slowed for (SafetyConfig::people_*); a track that never moved is just an obstacle.
  bool has_moved = false;
  // Set by Node: a person who stopped, was mapped and lost their track, kept at their last
  // spot for slowing down and keeping clear only. The map already holds them as an obstacle.
  bool remembered = false;
  // One standard deviation of the predicted position and of the velocity (Kalman), for
  // widening the predicted spot the further ahead it is used (SafetyConfig::mover_sigmas).
  double position_sigma_m = 0.0;
  double velocity_sigma_mps = 0.0;
  double first_seen = 0.0;
  double last_seen = 0.0;
  bool moving = false;
  bool confirmed = false;  // seen in enough frames; a tentative one's velocity means little
  std::vector<Eigen::Vector2d> trail;  // where it was seen, oldest first
};

class ObstacleTracker {
 public:
  explicit ObstacleTracker(TrackerConfig config = {});

  // One frame's returns that landed on free space, in odom, at `stamp`.
  void update(double stamp, const std::vector<Vec3>& returns);

  // Confirmed tracks predicted to `now`; with `moving_only`, only those moving.
  std::vector<TrackedObstacle> tracks(double now, bool moving_only = true) const;

  // Every track, tentative ones too, predicted to `now`.
  std::vector<TrackedObstacle> all(double now) const;

  // Whether a confirmed moving track, predicted to `stamp`, accounts for a return at `p`:
  // such returns are a passing body, not structure, and stay out of the map.
  bool explains(const Vec3& p, double stamp) const;

  // Tracks dropped since the last call, predicted to when they were dropped.
  std::vector<TrackedObstacle> take_lost();

  const TrackerConfig& config() const { return config_; }

 private:
  struct Track {
    TrackedObstacle out;
    Eigen::Vector4d x = Eigen::Vector4d::Zero();  // x, y, vx, vy
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
    double stamp = 0.0;
    int hits = 0;
    // Bounds of each detection in the trail, oldest first.
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> boxes;
  };
  struct Detection {
    Eigen::Vector2d centre;
    double radius_m;
    Eigen::Vector2d lo, hi;
    double z_low, z_high;
  };

  std::vector<Detection> cluster(const std::vector<Vec3>& returns) const;
  void predict(Track& t, double to) const;
  void correct(Track& t, const Eigen::Vector2d& z) const;
  TrackedObstacle predicted(const Track& t, double to) const;
  bool confirmed(const Track& t) const { return t.hits >= config_.confirm_hits; }
  bool edges_shifted(const Track& t) const;

  TrackerConfig config_;
  std::vector<Track> tracks_;
  std::vector<TrackedObstacle> lost_;
  int next_id_ = 1;
};

}  // namespace argus::node

#pragma once

#include <condition_variable>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "argus/capabilities/capabilities.hpp"
#include "argus/node/body.hpp"
#include "argus/node/local_map.hpp"
#include "argus/node/mission.hpp"
#include "argus/node/motion.hpp"
#include "argus/node/planner.hpp"
#include "argus/node/ray_pattern.hpp"
#include "argus/node/safety.hpp"
#include "argus/node/view.hpp"

namespace argus::node {

struct NodeConfig {
  // Window of the local map. Left at zero, it is sized from the body.
  LocalMapConfig map{0.1, 0.0, 0.0, 0.0};
  RayPatternOptions rays;
  PlannerConfig planner;
  SafetyConfig safety;
  // Planar bodies: hits lower than this above the ground are the ground itself.
  double min_obstacle_height_m = 0.05;
  // Planar bodies: a return this far or more below the floor is a drop (a stair down, a pit,
  // a dock edge). Where its ray crossed the floor is marked as an obstacle; 0 turns this off.
  double drop_depth_m = 0.1;
  // Rays with no return clear the space up to the sensor's max range.
  bool no_return_is_free = true;
  // Sensor health. A frame is refused, so it neither changes the map nor counts as fresh
  // (the sensor goes stale and the supervisor stops motion only it covers), when its stamp
  // does not move forward or runs ahead of odometry, when nearly every ray is invalid, or
  // when it repeats the last frame exactly while the robot has moved this far (a frozen
  // driver; a still robot in a still world may see identical frames legitimately).
  double max_invalid_share = 0.95;
  double frozen_motion_m = 0.05;
  double frozen_turn_rad = 0.05;
  // Odometry cross-check: each range sensor's frame is matched against a keyframe of its
  // own taken this far back in travel (correlative scan matching, Olson 2009). The shift
  // and turn that best line the two up are what odometry got wrong over that stretch.
  // Mismatches over the limits in odometry_strikes checks in a row make odometry suspect
  // (flag odometry_suspect, speed capped by SafetyConfig::odometry_suspect_speed_mps); as
  // many checks that agree clear it. A corridor gives no clear best match: no verdict.
  bool check_odometry = true;
  double odometry_check_travel_m = 0.75;
  double odometry_check_turn_rad = 0.25;
  // Also this long after the keyframe, travel or not: a gyro drifts at rest too.
  double odometry_check_period_s = 2.0;
  double odometry_max_shift_m = 0.075;
  double odometry_max_turn_rad = 0.035;
  int odometry_strikes = 2;
  double odometry_history_s = 2.0;
  // How long without getting closer to the waypoint before reporting it.
  double no_progress_s = 10.0;
  // How long the local map must show no way to the waypoint before reporting a dead end.
  double dead_end_s = 2.0;
  // How long Argo may find no clear motion toward its waypoint (or find the waypoint itself
  // blocked) before it stops and reports it, so the hub can route it another way. Argo does
  // not look for another route itself (decisions, "Argo's job").
  double stuck_report_s = 2.0;
  // Spacing of the trail Argo keeps for returning home without the hub.
  double breadcrumb_spacing_m = 0.5;
  size_t max_breadcrumbs = 4000;
  // Things moving near the robot (tracker.hpp), for every body: drones compare heights.
  bool track_movers = true;
  TrackerConfig tracker;
  // Something that was moving and stops on the way ahead may set off again in any direction:
  // Argo holds still this long, then goes around it (or on, if it moved away).
  double yield_wait_s = 3.0;
  double yield_lookahead_m = 3.0;
  // While it is still new to standing still, it is kept this much farther away than its size.
  double restart_margin_m = 0.3;
  double recently_moved_s = 10.0;
  // Something standing this long where space was ever-free stops being a passer-by and joins
  // the map, so the robot plans around it (Dynablox's occupancy counter, in seconds).
  double static_after_s = 3.0;
  // Plan inside tick() (deterministic, for the simulator and tests), or on a thread of its
  // own so a slow plan never holds up the control tick (the argo binary). Threaded, the
  // tick uses the latest finished plan; the supervisor's lease stops the robot if planning
  // falls behind, and range frames that arrive while a plan reads the map wait for it.
  bool plan_in_tick = true;
};

struct TickResult {
  Command command;
  uint32_t safety_flags = 0;
  Twist proposed;
  Proposal proposal;
  std::string target_id;
  std::vector<Event> events;
};

// Argo's core: everything on the robot between drivers and the robot controller, as a
// deterministic step API. Feed odometry and range frames as they arrive, hand it a plan, and
// call tick() at the control rate.
class Node {
 public:
  explicit Node(capabilities::Profile profile, NodeConfig config = {});
  ~Node();
  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  void on_odometry(const Odometry& odom);
  // Returns false when the frame is dropped: unknown sensor, wrong size, no pose for its time,
  // or refused by the health checks (NodeConfig, sensor_faults()).
  bool on_range_frame(const RangeFrame& frame);
  // Sensors whose latest frame was refused, with the reason. Empty when all are healthy.
  std::map<std::string, std::string> sensor_faults() const;
  // The odometry cross-check (NodeConfig::check_odometry): whether odometry is suspect, and
  // the last verdict's shift (m) and turn (rad), how many checks gave a verdict.
  struct OdometryCheck {
    bool suspect = false;
    double shift_m = 0.0;
    double turn_rad = 0.0;
    int verdicts = 0;
    // Every comparison, conclusive or not, and the last one's match counts.
    int checks = 0;
    int last_zero = 0;
    int last_best = 0;
    int last_points = 0;
  };
  const OdometryCheck& odometry_check() const { return odom_check_; }
  // Wall-clock time of each stage of frame integration and of the tick, in ms, one entry per
  // call, for argo_bench. Off unless `enabled`; then it costs a clock read per stage.
  struct StageTimes {
    bool enabled = false;
    std::map<std::string, std::vector<double>> ms;
  };
  StageTimes& stage_times() { return stage_times_; }
  bool set_plan(const Plan& plan, double now);
  TickResult tick(double now);

  // Setup motion tests: a velocity to try in place of the planner's until `until`. The safety
  // supervisor checks it like any other command.
  void request_velocity(const Twist& velocity, double until) {
    requested_ = velocity;
    requested_until_ = until;
  }
  bool velocity_requested(double now) const { return now <= requested_until_; }

  // Hub requests. Cancelling with `return_home` sends Argo back along its own trail.
  void cancel_plan(uint64_t generation, bool return_home, double now);
  void set_paused(bool paused, double now);
  void return_home(double now);

  // Link handling, driven by the session layer. A speed cap of 0 means none.
  void set_speed_cap(double mps) { speed_cap_ = mps; }
  double speed_cap() const { return speed_cap_; }
  // Runs `action` until cleared; hub plans are refused meanwhile.
  void enter_failsafe(capabilities::LinkLostAction action, double now);
  void clear_failsafe(double now);
  bool failsafe_active() const { return failsafe_active_; }

  uint64_t plan_generation() const { return mission_.generation(); }
  bool has_odometry() const { return have_odom_; }
  const Odometry& odometry() const { return odom_; }
  // Local map evidence since the last call; empty unless NodeConfig::map.record_evidence.
  std::vector<CellDelta> take_evidence() { return map_.take_evidence(); }
  const std::vector<Vec3>& breadcrumbs() const { return crumbs_; }

  const capabilities::Profile& profile() const { return profile_; }
  const capabilities::Capabilities& capabilities() const { return caps_; }
  const LocalMap& map() const { return map_; }
  const std::vector<Vec3>& path() const { return path_; }
  // Whether the last tick ran the planner (always false with a planning thread).
  bool last_tick_planned() const { return planned_in_tick_; }
  // What the supervisor was given as moving (or recently moving) on the last tick.
  const std::vector<TrackedObstacle>& movers() const { return movers_; }
  const RayPattern& ray_pattern(const std::string& sensor_id) const;
  const std::vector<double>& self_hits(const std::string& sensor_id) const;
  // Sensor pose in the base frame.
  Eigen::Isometry3d sensor_mount(const std::string& sensor_id) const;
  std::vector<std::string> range_sensor_ids() const;
  double control_period_s() const { return control_period_; }
  double command_lease_s() const { return supervisor_->lease_s(); }

 private:
  struct RangeSensor {
    const capabilities::SensorSpec* spec = nullptr;
    Eigen::Isometry3d mount = Eigen::Isometry3d::Identity();
    RayPattern pattern;
    std::vector<double> self_hits;
    ViewLayout view;
  };

  bool pose_at(double stamp, Pose4& pose, double* roll = nullptr, double* pitch = nullptr) const;
  void view_band(const Vec3& at, double& lo, double& hi) const;
  const RangeSensor& range_sensor(const std::string& id) const;
  void watch_progress(const Waypoint* target, uint32_t flags, double now, std::vector<Event>& events);
  void watch_dead_end(const Waypoint* target, double now, std::vector<Event>& events);
  Plan home_plan() const;
  bool clear_line(const Vec3& a, const Vec3& b) const;
  std::vector<Vec3> shortcut(const std::vector<Vec3>& trail) const;
  Plan land_plan() const;

  capabilities::Profile profile_;
  capabilities::Capabilities caps_;
  NodeConfig config_;
  double control_period_ = 0.02;
  double ground_z_ = 0.0;
  LocalMap map_;
  // Cells closer than this to the body are checked against each frame's view.
  double view_radius_m_ = 0.0;
  std::map<std::string, RangeSensor> sensors_;
  SensorClock clock_;
  std::deque<Odometry> history_;
  Odometry odom_;
  bool have_odom_ = false;

  std::unique_ptr<BodyVolume> body_;
  std::unique_ptr<MotionModel> motion_;
  std::unique_ptr<SafetySupervisor> supervisor_;
  std::unique_ptr<LocalPlanner> planner_;
  Mission mission_;

  // Moving obstacles: the tracker, cells taking dynamic returns (since when, last when), what
  // the supervisor is given each tick, and the yield (who it is for, until when).
  ObstacleTracker tracker_;
  struct DynamicCell {
    double since;
    double last;
  };
  std::map<std::tuple<int, int, int>, DynamicCell> dynamic_cells_;
  std::vector<TrackedObstacle> movers_;
  std::map<int, double> last_moving_;
  std::map<int, bool> yielded_for_;
  std::vector<std::pair<Eigen::Vector2d, double>> yielded_at_;  // where and when it waited
  std::map<int, std::pair<TrackedObstacle, double>> people_;   // last seen as having moved, when
  int yield_id_ = 0;
  double yield_until_ = -1e9;
  // A pause for something in the way that the hub has not been told about yet.
  bool pause_pending_ = false;
  Vec3 pause_at_ = Vec3::Zero();
  // Since when Argo has found no clear motion toward the current waypoint, and for which
  // waypoint it last reported that.
  double stuck_since_ = -1.0;
  std::string stuck_reported_;
  // Where the robot was when it set off for the current waypoint: the start of the hub's leg.
  std::string leg_key_;
  Vec3 leg_start_ = Vec3::Zero();
  std::vector<Vec3> route_for(const Waypoint& target, const Waypoint* following);
  bool tracking() const;
  bool reaches_height(const TrackedObstacle& o, double z, double margin) const;
  void forget_lost(double stamp);
  bool update_yield(double now);

  Proposal proposal_;
  std::vector<Vec3> path_;
  bool planned_in_tick_ = false;
  double last_plan_time_ = -1e9;

  // Planning thread (NodeConfig::plan_in_tick false). A job is a copy of everything the
  // planner reads except the map, which it reads under a shared lock; only frame
  // integration writes the map, and it never waits for the lock: frames wait instead.
  struct PlanJob {
    Waypoint target;
    std::optional<Waypoint> following;
    Odometry odom;
    SensorClock clock;
    std::vector<TrackedObstacle> movers;
    double now = 0.0;
    SafetySupervisor supervisor;
    bool odometry_suspect = false;
    std::vector<Vec3> route;
  };
  struct PlanDone {
    Waypoint target;
    Proposal proposal;
    std::vector<Vec3> path;
  };
  void plan_worker();
  bool integrate_frame(const RangeFrame& frame);
  struct SensorHealth {
    double last_stamp = -std::numeric_limits<double>::infinity();
    std::vector<float> last_ranges;
    bool repeating = false;
    Pose4 repeat_from;
    std::string fault;
  };
  bool frame_healthy(const RangeSensor& s, const RangeFrame& frame, const Pose4& pose, SensorHealth& h) const;
  std::map<std::string, SensorHealth> health_;
  struct OdometryKeyframe {
    Pose4 pose;
    double stamp = 0.0;
    std::vector<Vec3> points;  // static returns, in odom
  };
  void cross_check_odometry(const std::string& sensor_id, double stamp, const Pose4& pose, const std::vector<Vec3>& ends,
                            const std::vector<bool>& hits, const std::vector<bool>& drops);
  std::map<std::string, OdometryKeyframe> odom_keys_;
  OdometryCheck odom_check_;
  StageTimes stage_times_;
  int odom_strikes_ = 0;
  int odom_agreements_ = 0;
  void mark_drop_edges(const RangeSensor& s, const Eigen::Isometry3d& to_odom, const Vec3& origin, double floor_z,
                       double ground, const std::vector<size_t>& drop_rays, std::vector<Vec3>& ends,
                       std::vector<bool>& hits, std::vector<bool>& drops) const;
  void flush_deferred_frames();
  std::shared_mutex map_mutex_;
  std::mutex job_mutex_;
  std::condition_variable job_cv_;
  std::optional<PlanJob> job_;
  bool job_running_ = false;
  std::optional<PlanDone> done_;
  bool stop_worker_ = false;
  std::thread worker_;
  std::deque<RangeFrame> deferred_frames_;
  Twist requested_;
  double requested_until_ = -1e9;
  std::vector<Event> pending_;
  uint32_t last_flags_ = 0;

  std::string progress_target_;
  double best_distance_ = 0.0;
  double last_progress_ = 0.0;
  std::string dead_end_target_;
  double exhausted_since_ = -1.0;
  bool dead_end_reported_ = false;

  double speed_cap_ = 0.0;
  bool paused_ = false;
  bool failsafe_active_ = false;
  capabilities::LinkLostAction failsafe_action_ = capabilities::LinkLostAction::Stop;
  std::vector<Vec3> crumbs_;
  // Where the current creep stretch began, and the way back there once it ran out with
  // nothing the body can see to drive on.
  Vec3 creep_start_ = Vec3::Zero();
  bool retracing_ = false;
  std::string retrace_for_;
};

}  // namespace argus::node

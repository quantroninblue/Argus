#pragma once

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "argus/capabilities/capabilities.hpp"
#include "argus/node/body.hpp"
#include "argus/node/local_map.hpp"
#include "argus/node/mission.hpp"
#include "argus/node/motion.hpp"
#include "argus/node/planner.hpp"
#include "argus/node/ray_pattern.hpp"
#include "argus/node/safety.hpp"

namespace argus::node {

struct NodeConfig {
  // Window of the local map. Left at zero, it is sized from the body.
  LocalMapConfig map{0.1, 0.0, 0.0, 0.0};
  RayPatternOptions rays;
  PlannerConfig planner;
  SafetyConfig safety;
  // Planar bodies: hits lower than this above the ground are the ground itself.
  double min_obstacle_height_m = 0.05;
  // Rays with no return clear the space up to the sensor's max range.
  bool no_return_is_free = true;
  double odometry_history_s = 2.0;
  // How long without getting closer to the waypoint before reporting it.
  double no_progress_s = 10.0;
  // How long the local map must show no way to the waypoint before reporting a dead end.
  double dead_end_s = 2.0;
  // Spacing of the trail Argo keeps for returning home without the hub.
  double breadcrumb_spacing_m = 0.5;
  size_t max_breadcrumbs = 4000;
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
  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  void on_odometry(const Odometry& odom);
  // Returns false when the frame is dropped: unknown sensor, wrong size, or no pose for its time.
  bool on_range_frame(const RangeFrame& frame);
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
  const std::vector<Vec3>& path() const { return planner_->path(); }
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
  };

  bool pose_at(double stamp, Pose4& pose) const;
  const RangeSensor& range_sensor(const std::string& id) const;
  void watch_progress(const Waypoint* target, uint32_t flags, double now, std::vector<Event>& events);
  void watch_dead_end(const Waypoint* target, double now, std::vector<Event>& events);
  Plan home_plan() const;
  Plan land_plan() const;

  capabilities::Profile profile_;
  capabilities::Capabilities caps_;
  NodeConfig config_;
  double control_period_ = 0.02;
  double ground_z_ = 0.0;
  LocalMap map_;
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

  Proposal proposal_;
  double last_plan_time_ = -1e9;
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

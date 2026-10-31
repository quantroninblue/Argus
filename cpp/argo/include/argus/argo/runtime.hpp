#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "argus/node/grpc_link.hpp"
#include "argus/node/session.hpp"

namespace argus::argo {

using node::Command;
using node::Odometry;
using node::RangeFrame;

// Where drivers leave what they read, from any thread. The control thread takes everything at
// the start of each tick. Odometry is kept in order (Argo interpolates poses between samples);
// range frames keep only the newest per sensor, so a slow tick never builds a backlog.
class Inbox {
 public:
  void push(const Odometry& odom);
  void push(RangeFrame frame);
  void take(std::vector<Odometry>& odom, std::vector<RangeFrame>& frames);

 private:
  static constexpr size_t kMaxOdometry = 256;
  std::mutex mutex_;
  std::deque<Odometry> odom_;
  std::map<std::string, RangeFrame> frames_;
};

// The robot side of the runtime. A driver reads sensors on its own threads into the inbox and
// passes commands to the robot controller. send() is called from the control thread and must
// not block.
class Driver {
 public:
  virtual ~Driver() = default;
  virtual void start(Inbox& inbox) = 0;
  virtual void send(const Command& command) = 0;
  virtual void stop() {}
};

struct RuntimeConfig {
  node::NodeConfig node;
  node::SessionConfig session;
  // An empty target runs Argo with no hub (bench tests).
  node::GrpcLinkConfig link;
  // On shutdown, stop commands keep going out for this long.
  double stop_repeat_s = 0.5;
  // Setup mode: no hub; Argo drives only velocities requested through request_velocity(),
  // each checked by the supervisor, and sends nothing at all between requests, so the robot's
  // own controller timeout can be tested.
  bool setup_mode = false;
};

struct TickStats {
  uint64_t ticks = 0;
  // Ticks that took longer than the control period.
  uint64_t overruns = 0;
  double worst_s = 0.0;
};

// Argo on a robot: the core and its hub session stepped at the control rate on one thread,
// with the network on GrpcLink's own thread. Neither the link nor the drivers can hold up a
// tick, so a stalled network never delays a stop.
class Runtime {
 public:
  using Clock = std::function<double()>;
  using Log = std::function<void(const std::string&)>;

  Runtime(capabilities::Profile profile, RuntimeConfig config, Driver& driver, Clock clock = wall_clock,
          Log log = log_to_stderr);
  ~Runtime();

  // Runs the control loop until stop() is called, then sends stop commands and returns.
  void run();
  // Safe from any thread and from a signal handler.
  void stop() { stop_ = true; }

  // Setup mode: drive `velocity` for the next `for_s` seconds. Safe from any thread.
  void request_velocity(const node::Twist& velocity, double for_s);

  // One control tick at `now`. run() calls this; tests can too.
  node::TickResult step(double now);

  const TickStats& stats() const { return stats_; }
  // Safety flags of the latest tick. Safe from any thread.
  uint32_t flags() const { return last_flags_; }
  node::Node& core() { return *node_; }
  node::Session& session() { return *session_; }
  double period_s() const { return node_->control_period_s(); }

  // Seconds since the Unix epoch, the time base of ROS 2 message stamps and of the protocol.
  static double wall_clock();
  static void log_to_stderr(const std::string& line);

 private:
  void poll_link(double now);
  void report(const node::TickResult& result, double now);

  RuntimeConfig config_;
  Driver& driver_;
  Clock clock_;
  Log log_;
  std::unique_ptr<node::Node> node_;
  std::unique_ptr<node::Session> session_;
  std::unique_ptr<node::GrpcLink> link_;
  Inbox inbox_;
  std::atomic<bool> stop_{false};
  std::mutex request_mutex_;
  bool request_pending_ = false;
  node::Twist request_;
  double request_for_s_ = 0.0;
  TickStats stats_;
  std::atomic<uint32_t> last_flags_{0};
  std::vector<Odometry> odom_buffer_;
  std::vector<RangeFrame> frame_buffer_;
};

}  // namespace argus::argo

#include "argus/argo/runtime.hpp"

#include <cmath>

#include <chrono>
#include <cstdio>
#include <sstream>
#include <thread>

namespace argus::argo {

void Inbox::push(const Odometry& odom) {
  std::lock_guard<std::mutex> lock(mutex_);
  odom_.push_back(odom);
  while (odom_.size() > kMaxOdometry) {
    odom_.pop_front();
  }
}

void Inbox::push(RangeFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto& slot = frames_[frame.sensor_id];
  if (frame.stamp >= slot.stamp) {
    slot = std::move(frame);
  }
}

void Inbox::take(std::vector<Odometry>& odom, std::vector<RangeFrame>& frames) {
  odom.clear();
  frames.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  odom.assign(odom_.begin(), odom_.end());
  odom_.clear();
  for (auto& [id, frame] : frames_) {
    if (!frame.ranges.empty()) {
      frames.push_back(std::move(frame));
      frame = RangeFrame{};
    }
  }
}

double Runtime::wall_clock() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

void Runtime::log_to_stderr(const std::string& line) {
  std::fprintf(stderr, "%s\n", line.c_str());
}

// Every line starts with the runtime's own clock: simulated time in a simulation, so a run's
// log lines up with its recording, and the robot's time on a robot.
void Runtime::say(const std::string& line) {
  char stamp[32];
  std::snprintf(stamp, sizeof stamp, "%.3f ", clock_());
  log_(stamp + line);
}

Runtime::Runtime(capabilities::Profile profile, RuntimeConfig config, Driver& driver, Clock clock, Log log)
    : config_(std::move(config)), driver_(driver), clock_(std::move(clock)), log_(std::move(log)) {
  config_.node.map.record_evidence = true;
  // A slow plan must never hold up the control tick on the robot.
  config_.node.plan_in_tick = false;
  node_ = std::make_unique<node::Node>(std::move(profile), config_.node);
  session_ = std::make_unique<node::Session>(*node_, config_.session);
  if (!config_.link.target.empty() && !config_.setup_mode) {
    link_ = std::make_unique<node::GrpcLink>(config_.link);
  }
}

Runtime::~Runtime() = default;

void Runtime::poll_link(double now) {
  if (!link_) {
    return;
  }
  // Messages and stream changes are handled in the order they happened: a hub that rejects
  // the session sends its reason and then closes.
  node::GrpcLink::Update update = link_->poll();
  size_t next = 0;
  for (const auto& t : update.transitions) {
    for (; next < t.after && next < update.received.size(); ++next) {
      session_->receive(update.received[next], now);
    }
    if (t.connected) {
      session_->on_connected(now);
    } else {
      session_->on_disconnected(now);
    }
  }
  for (; next < update.received.size(); ++next) {
    session_->receive(update.received[next], now);
  }
}

void Runtime::request_velocity(const node::Twist& velocity, double for_s) {
  std::lock_guard<std::mutex> lock(request_mutex_);
  request_ = velocity;
  request_for_s_ = for_s;
  request_pending_ = true;
}

node::TickResult Runtime::step(double now) {
  {
    std::lock_guard<std::mutex> lock(request_mutex_);
    if (request_pending_ && config_.setup_mode) {
      node_->request_velocity(request_, now + request_for_s_);
    }
    request_pending_ = false;
  }
  inbox_.take(odom_buffer_, frame_buffer_);
  for (const Odometry& o : odom_buffer_) {
    node_->on_odometry(o);
  }
  for (const RangeFrame& f : frame_buffer_) {
    node_->on_range_frame(f);
  }
  poll_link(now);
  node::TickResult result = session_->step(now);
  if (!config_.setup_mode || node_->velocity_requested(now)) {
    driver_.send(result.command);
  }
  std::vector<std::string> out = session_->take_outgoing();
  if (link_) {
    link_->send(out);
  }
  report(result, now);
  return result;
}

void Runtime::report(const node::TickResult& result, double now) {
  for (const node::Event& e : result.events) {
    std::ostringstream line;
    line << "event " << node::to_string(e.kind);
    if (!e.waypoint_id.empty()) line << " waypoint=" << e.waypoint_id;
    if (e.cause != node::FailureCause::None) line << " cause=" << node::to_string(e.cause);
    if (!e.detail.empty()) line << " detail=\"" << e.detail << "\"";
    say(line.str());
  }
  // Safety flags only when they change, so a robot standing still does not fill the log.
  if (result.safety_flags != last_flags_) {
    std::ostringstream line;
    line << "flags";
    for (const std::string& name : node::flag_names(result.safety_flags)) line << " " << name;
    if (result.safety_flags == 0) line << " none";
    say(line.str());
    last_flags_ = result.safety_flags;
  }
  // While Argo has somewhere to go but its planner proposes standing still, say why, once
  // when that starts or changes and then every few seconds: the evidence that tells a planner
  // stalled with clear options from a map with no way through.
  const node::Proposal& p = result.proposal;
  const bool still = p.velocity.linear.norm() < 1e-3 && std::abs(p.velocity.yaw_rate) < 1e-3;
  if (p.has_target && still) {
    std::ostringstream why;
    why << "path=" << (p.path_exhausted ? "none" : p.path_reaches_target ? "reaches" : "partial")
        << " target_blocked=" << (p.target_blocked ? 1 : 0) << " candidates=" << p.candidates
        << " clear=" << p.clear_rollouts << " checked=" << p.admission_checks;
    if (why.str() != last_stall_ || now - last_stall_at_ >= kStallRepeatS) {
      say("planner still " + why.str());
      last_stall_ = why.str();
      last_stall_at_ = now;
    }
  } else {
    last_stall_.clear();
  }
}

void Runtime::run() {
  using namespace std::chrono;
  const auto period = duration_cast<steady_clock::duration>(duration<double>(period_s()));
  driver_.start(inbox_);
  say("start period_s=" + std::to_string(period_s()) + (link_ ? " hub=" + config_.link.target : " hub=none"));
  auto next = steady_clock::now();
  while (!stop_) {
    const auto begin = steady_clock::now();
    step(clock_());
    const double took = duration<double>(steady_clock::now() - begin).count();
    ++stats_.ticks;
    stats_.worst_s = std::max(stats_.worst_s, took);
    if (took > period_s()) {
      ++stats_.overruns;
    }
    next += period;
    if (next < steady_clock::now()) {
      next = steady_clock::now();  // after an overrun, start again rather than catch up
    }
    std::this_thread::sleep_until(next);
  }
  // Leave the robot stopped: its controller holds the last command until its own timeout.
  const auto until = steady_clock::now() + duration_cast<steady_clock::duration>(duration<double>(config_.stop_repeat_s));
  while (steady_clock::now() < until) {
    Command stop;
    stop.stamp = clock_();
    stop.stop = true;
    driver_.send(stop);
    std::this_thread::sleep_for(period);
  }
  driver_.stop();
  std::ostringstream line;
  line << "stop ticks=" << stats_.ticks << " overruns=" << stats_.overruns << " worst_ms=" << stats_.worst_s * 1e3;
  say(line.str());
}

}  // namespace argus::argo

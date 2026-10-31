#pragma once

#include <cstdint>
#include <deque>
#include <string>

namespace argus::node {

enum class LinkState { Connected, Degraded, Lost };

std::string to_string(LinkState state);

struct LinkConfig {
  // From the profile's failsafe.link_lost_after_s.
  double lost_after_s = 3.0;
  double heartbeat_period_s = 0.2;
  // Degraded when the round trip or the loss over the recent window exceeds these, or when
  // the hub has been silent for `silent_heartbeats` periods.
  double degraded_rtt_s = 0.5;
  double degraded_loss = 0.2;
  int silent_heartbeats = 3;
  double loss_window_s = 5.0;
  // How long the link must look healthy again before a lost link counts as back.
  double recover_after_s = 1.0;
};

// Tracks the link to the hub from heartbeats alone, on Argo's own clock: each heartbeat Argo
// sends is echoed back by the hub, which gives the round trip without synchronised clocks.
class LinkMonitor {
 public:
  explicit LinkMonitor(LinkConfig config) : config_(config) {}

  // A session was accepted. Before the first one the link counts as lost. A dropped
  // connection is not lost by itself: only `lost_after_s` without heartbeats is, so a quick
  // reconnect does not trigger the failsafe.
  void on_session_start(double now);
  void on_sent(uint64_t seq, double now);
  // A heartbeat from the hub, echoing `echo_seq` (0 if none yet).
  void on_heartbeat(uint64_t echo_seq, double now);

  LinkState update(double now);

  LinkState state() const { return state_; }
  double round_trip_s() const { return rtt_s_; }
  double loss_fraction() const { return loss_; }
  const LinkConfig& config() const { return config_; }

 private:
  struct Sent {
    uint64_t seq = 0;
    double at = 0.0;
    bool echoed = false;
  };

  LinkConfig config_;
  LinkState state_ = LinkState::Lost;
  bool started_ = false;
  double last_heard_ = -1e9;
  double healthy_since_ = -1e9;
  double rtt_s_ = 0.0;
  double loss_ = 0.0;
  std::deque<Sent> sent_;
};

}  // namespace argus::node

#include "argus/node/link.hpp"

#include <algorithm>

namespace argus::node {

std::string to_string(LinkState state) {
  switch (state) {
    case LinkState::Connected:
      return "connected";
    case LinkState::Degraded:
      return "degraded";
    case LinkState::Lost:
      return "lost";
  }
  return "unknown";
}

void LinkMonitor::on_session_start(double now) {
  started_ = true;
  last_heard_ = now;
  sent_.clear();
  loss_ = 0.0;
  rtt_s_ = 0.0;
}

void LinkMonitor::on_sent(uint64_t seq, double now) {
  sent_.push_back({seq, now, false});
  while (!sent_.empty() && sent_.front().at < now - config_.loss_window_s) {
    sent_.pop_front();
  }
}

void LinkMonitor::on_heartbeat(uint64_t echo_seq, double now) {
  if (!started_) {
    return;
  }
  last_heard_ = now;
  for (Sent& s : sent_) {
    if (s.seq == echo_seq && !s.echoed) {
      s.echoed = true;
      rtt_s_ = now - s.at;
    }
  }
}

LinkState LinkMonitor::update(double now) {
  // Loss over heartbeats old enough that their echo should have arrived.
  const double grace = 2.0 * config_.heartbeat_period_s + std::max(rtt_s_, config_.heartbeat_period_s);
  int due = 0;
  int missing = 0;
  for (const Sent& s : sent_) {
    if (s.at < now - grace) {
      ++due;
      missing += s.echoed ? 0 : 1;
    }
  }
  loss_ = due > 0 ? static_cast<double>(missing) / due : 0.0;

  const double silent = now - last_heard_;
  LinkState health;
  if (!started_ || silent > config_.lost_after_s) {
    health = LinkState::Lost;
  } else if (silent > config_.silent_heartbeats * config_.heartbeat_period_s ||
             rtt_s_ > config_.degraded_rtt_s || loss_ > config_.degraded_loss) {
    health = LinkState::Degraded;
  } else {
    health = LinkState::Connected;
  }

  if (health == LinkState::Lost) {
    healthy_since_ = -1e9;
    state_ = LinkState::Lost;
    return state_;
  }
  if (state_ == LinkState::Lost) {
    // Coming back: wait until heartbeats have flowed for a while before trusting the link.
    if (healthy_since_ < -1e8) {
      healthy_since_ = now;
    }
    if (now - healthy_since_ < config_.recover_after_s) {
      return state_;
    }
  }
  state_ = health;
  return state_;
}

}  // namespace argus::node

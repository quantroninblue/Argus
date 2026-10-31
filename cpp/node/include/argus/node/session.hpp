#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Geometry>

#include "argus/node/link.hpp"
#include "argus/node/node.hpp"
#include "argus/protocol/v1/link.pb.h"

namespace argus::node {

struct SessionConfig {
  std::string node_id;
  std::string software_version = "0.1.0";
  std::string profile_hash;
  std::string calibration_version;
  double heartbeat_period_s = 0.2;
  double telemetry_period_s = 0.1;
  double evidence_period_s = 0.5;
  double hello_retry_s = 1.0;
  // Speed cap while the link is degraded, as a fraction of the body's top speed.
  double degraded_speed_fraction = 0.5;
  size_t max_buffered_events = 200;
  // Evidence kept while it cannot be sent: changed cells (merged per cell) and sent messages
  // the hub has not acknowledged yet. Past these the oldest is dropped.
  size_t max_pending_cells = 400000;
  size_t max_unacked_bytes = 4u << 20;
  // Largest single evidence message.
  size_t max_evidence_message_bytes = 1u << 20;
  // Link thresholds other than lost_after_s, which comes from the profile.
  LinkConfig link;
};

// Map evidence on its way to the hub. Cell changes merge per cell until they are sent, so a
// cell that changes ten times during an outage costs one entry. Sent messages stay until the
// hub acknowledges their sequence number and are resent if it does not; the hub applies each
// number once. The same scheme as TCP's retransmission, and with the hub's budget below it,
// Breakwater's credits (Cho et al., OSDI 2020): what the hub cannot take is never sent.
class EvidenceOutbox {
 public:
  struct Batch {
    uint64_t seq = 0;
    std::string payload;
  };

  EvidenceOutbox(uint64_t stream, size_t max_pending_cells, size_t max_unacked_bytes);

  void add(const std::vector<CellDelta>& deltas);
  // The next message from the pending cells, whole tiles only, at most `max_bytes`; nothing
  // when no cell is pending or the first tile does not fit.
  std::optional<Batch> next(size_t max_bytes, double now);
  void ack(uint64_t seq);
  // Sent messages not acknowledged within `timeout_s`, oldest first, up to `max_bytes`; their
  // timers restart.
  std::vector<Batch> due(double now, double timeout_s, size_t max_bytes);

  uint64_t stream() const { return stream_; }
  // Bytes waiting: pending cells (as encoded) plus unacknowledged messages.
  size_t backlog_bytes() const;
  size_t pending_cells() const { return pending_cells_; }
  size_t unacked() const { return unacked_.size(); }
  uint64_t dropped_cells() const { return dropped_cells_; }

 private:
  using Key = std::tuple<int, int, int>;
  struct Sent {
    uint64_t seq;
    std::string payload;
    double at;
  };

  uint64_t stream_;
  size_t max_pending_cells_;
  size_t max_unacked_bytes_;
  std::map<Key, std::map<Key, int>> pending_;  // tile -> cell -> summed delta
  size_t pending_cells_ = 0;
  std::deque<Sent> unacked_;
  size_t unacked_bytes_ = 0;
  uint64_t seq_ = 0;
  uint64_t dropped_cells_ = 0;
};

// Argo's side of the hub link, without a transport: feed it what arrives, call step() at the
// control rate, and send what take_outgoing() returns. The transport (gRPC, or the simulator's
// channel) only reports when its stream opens and closes.
//
// It owns the link state and decides the failsafe: when no valid heartbeat has arrived for the
// profile's link_lost_after_s, Argo runs the profile's on_link_lost action until a new session
// has been healthy for a while.
class Session {
 public:
  Session(Node& node, SessionConfig config);

  void on_connected(double now);
  void on_disconnected(double now);
  // False if `bytes` is not a HubMessage.
  bool receive(const std::string& bytes, double now);
  void receive(const protocol::v1::HubMessage& message, double now);

  // One Argo control tick, plus link handling and the messages it produces.
  TickResult step(double now);
  std::vector<std::string> take_outgoing();

  bool connected() const { return connected_; }
  bool accepted() const { return accepted_; }
  bool rejected() const { return rejected_; }
  const std::string& reject_detail() const { return reject_detail_; }
  LinkState link_state() const { return link_.state(); }
  const LinkMonitor& link() const { return link_; }
  uint64_t map_epoch() const { return map_epoch_; }
  // Bytes of evidence sent so far, for bandwidth checks.
  uint64_t evidence_bytes() const { return evidence_bytes_; }
  const EvidenceOutbox& evidence() const { return outbox_; }
  // Evidence bytes per second the hub allows; 0 until it says.
  double evidence_budget() const { return budget_bps_; }

 private:
  void send(protocol::v1::NodeMessage& message);
  void send_hello(double now);
  void queue_event(protocol::v1::Event event);
  void handle_plan(const protocol::v1::MissionPlan& plan, double now);
  void on_link_change(LinkState from, LinkState to, double now);
  void send_evidence(double now);
  void send_evidence_message(const EvidenceOutbox::Batch& batch, double now);
  protocol::v1::Event node_event(const Event& e) const;
  double hub_to_node(int64_t unix_nanos) const;

  Node& node_;
  SessionConfig config_;
  LinkMonitor link_;
  bool ever_accepted_ = false;
  bool connected_ = false;
  bool accepted_ = false;
  bool rejected_ = false;
  std::string reject_detail_;
  double hello_sent_ = -1e9;
  uint64_t map_epoch_ = 0;
  int64_t clock_offset_nanos_ = 0;
  Eigen::Isometry3d map_to_odom_ = Eigen::Isometry3d::Identity();

  uint64_t seq_ = 0;
  uint64_t heartbeat_seq_ = 0;
  uint64_t hub_heartbeat_seq_ = 0;
  int64_t hub_heartbeat_sent_ = 0;
  double next_heartbeat_ = 0.0;
  double next_telemetry_ = 0.0;
  double next_evidence_ = 0.0;
  uint64_t evidence_bytes_ = 0;
  EvidenceOutbox outbox_;
  double budget_bps_ = 0.0;
  double budget_tokens_ = 0.0;
  double budget_at_ = -1.0;

  std::deque<protocol::v1::Event> events_;
  std::vector<std::string> outgoing_;
};

}  // namespace argus::node

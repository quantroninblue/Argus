#include "argus/node/session.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include "argus/grid/evidence.hpp"

namespace argus::node {

namespace pb = argus::protocol::v1;

namespace {

constexpr uint32_t kProtocolVersion = 1;

double top_speed(const capabilities::MotionCapability& m) {
  double v = 0.0;
  for (const auto* a : {&m.x, &m.y, &m.z}) {
    v = std::max({v, std::abs(a->min_mps), std::abs(a->max_mps)});
  }
  return v;
}

int64_t nanos(double seconds) { return static_cast<int64_t>(std::llround(seconds * 1e9)); }

void set_stamp(pb::Stamp* stamp, double seconds) { stamp->set_unix_nanos(nanos(seconds)); }

void set_vec(pb::Vector3* out, const Vec3& v) {
  out->set_x(v.x());
  out->set_y(v.y());
  out->set_z(v.z());
}

Vec3 vec(const pb::Vector3& v) { return Vec3(v.x(), v.y(), v.z()); }

void set_pose(pb::Pose* out, const Eigen::Isometry3d& t) {
  set_vec(out->mutable_position(), t.translation());
  const Eigen::Quaterniond q(t.linear());
  out->mutable_orientation()->set_x(q.x());
  out->mutable_orientation()->set_y(q.y());
  out->mutable_orientation()->set_z(q.z());
  out->mutable_orientation()->set_w(q.w());
}

Eigen::Isometry3d isometry(const pb::Pose& p) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  const auto& o = p.orientation();
  Eigen::Quaterniond q(o.w(), o.x(), o.y(), o.z());
  if (q.norm() < 1e-9) {
    q = Eigen::Quaterniond::Identity();
  }
  t.linear() = q.normalized().toRotationMatrix();
  t.translation() = vec(p.position());
  return t;
}

pb::FailureCause to_proto(FailureCause c) {
  switch (c) {
    case FailureCause::None:
      return pb::FAILURE_CAUSE_UNSPECIFIED;
    case FailureCause::DeadEndObserved:
      return pb::FAILURE_CAUSE_DEAD_END_OBSERVED;
    case FailureCause::SearchedNoGoal:
      return pb::FAILURE_CAUSE_SEARCHED_NO_GOAL;
    case FailureCause::TemporaryObstruction:
      return pb::FAILURE_CAUSE_TEMPORARY_OBSTRUCTION;
    case FailureCause::ControllerNoProgress:
      return pb::FAILURE_CAUSE_CONTROLLER_NO_PROGRESS;
    case FailureCause::LocalizationInvalid:
      return pb::FAILURE_CAUSE_LOCALIZATION_INVALID;
    case FailureCause::SensorInvalid:
      return pb::FAILURE_CAUSE_SENSOR_INVALID;
    case FailureCause::ComputeDeadlineMiss:
      return pb::FAILURE_CAUSE_COMPUTE_DEADLINE_MISS;
    case FailureCause::LinkLost:
      return pb::FAILURE_CAUSE_LINK_LOST;
  }
  return pb::FAILURE_CAUSE_UNSPECIFIED;
}

pb::LinkState to_proto(LinkState s) {
  switch (s) {
    case LinkState::Connected:
      return pb::LINK_STATE_CONNECTED;
    case LinkState::Degraded:
      return pb::LINK_STATE_DEGRADED;
    case LinkState::Lost:
      return pb::LINK_STATE_LOST;
  }
  return pb::LINK_STATE_UNSPECIFIED;
}

}  // namespace

namespace {

uint64_t new_stream_id() {
  std::random_device rd;
  return (static_cast<uint64_t>(rd()) << 32) ^ rd() ^ 1u;  // never 0
}

std::tuple<int, int, int> key(const Index3& i) { return {i.x(), i.y(), i.z()}; }

}  // namespace

EvidenceOutbox::EvidenceOutbox(uint64_t stream, size_t max_pending_cells, size_t max_unacked_bytes)
    : stream_(stream), max_pending_cells_(max_pending_cells), max_unacked_bytes_(max_unacked_bytes) {}

void EvidenceOutbox::add(const std::vector<CellDelta>& deltas) {
  for (const CellDelta& d : deltas) {
    auto& tile = pending_[key(grid::tile_of(d.index))];
    auto [it, inserted] = tile.try_emplace(key(d.index), 0);
    it->second += d.delta;
    pending_cells_ += inserted;
  }
  while (pending_cells_ > max_pending_cells_ && !pending_.empty()) {
    pending_cells_ -= pending_.begin()->second.size();
    dropped_cells_ += pending_.begin()->second.size();
    pending_.erase(pending_.begin());
  }
}

std::optional<EvidenceOutbox::Batch> EvidenceOutbox::next(size_t max_bytes, double now) {
  constexpr size_t kHeader = 4;
  constexpr size_t kTileHeader = 14;
  constexpr size_t kCell = 3;
  std::vector<CellDelta> cells;
  size_t bytes = kHeader;
  while (!pending_.empty()) {
    const auto& tile = pending_.begin()->second;
    const size_t size = kTileHeader + kCell * tile.size();
    if (bytes + size > max_bytes) {
      break;
    }
    bytes += size;
    for (const auto& [c, delta] : tile) {
      cells.push_back({Index3(std::get<0>(c), std::get<1>(c), std::get<2>(c)), delta});
    }
    pending_cells_ -= tile.size();
    pending_.erase(pending_.begin());
  }
  if (cells.empty()) {
    return std::nullopt;
  }
  Batch b{++seq_, grid::encode_evidence(cells)};
  unacked_.push_back({b.seq, b.payload, now});
  unacked_bytes_ += b.payload.size();
  while (unacked_bytes_ > max_unacked_bytes_ && unacked_.size() > 1) {
    unacked_bytes_ -= unacked_.front().payload.size();
    unacked_.pop_front();  // lost: the hub plans around the gap until the place is seen again
  }
  return b;
}

void EvidenceOutbox::ack(uint64_t seq) {
  while (!unacked_.empty() && unacked_.front().seq <= seq) {
    unacked_bytes_ -= unacked_.front().payload.size();
    unacked_.pop_front();
  }
}

std::vector<EvidenceOutbox::Batch> EvidenceOutbox::due(double now, double timeout_s, size_t max_bytes) {
  std::vector<Batch> out;
  size_t bytes = 0;
  for (Sent& s : unacked_) {
    if (now - s.at < timeout_s || bytes + s.payload.size() > max_bytes) {
      break;
    }
    bytes += s.payload.size();
    s.at = now;
    out.push_back({s.seq, s.payload});
  }
  return out;
}

size_t EvidenceOutbox::backlog_bytes() const {
  size_t bytes = unacked_bytes_;
  for (const auto& [t, cells] : pending_) {
    bytes += 14 + 3 * cells.size();
  }
  return bytes;
}

Session::Session(Node& node, SessionConfig config)
    : node_(node),
      config_(std::move(config)),
      link_(config_.link),
      outbox_(new_stream_id(), config_.max_pending_cells, config_.max_unacked_bytes) {
  LinkConfig link = config_.link;
  link.lost_after_s = node_.profile().failsafe.link_lost_after_s;
  link.heartbeat_period_s = config_.heartbeat_period_s;
  link_ = LinkMonitor(link);
}

double Session::hub_to_node(int64_t unix_nanos) const {
  return static_cast<double>(unix_nanos + clock_offset_nanos_) * 1e-9;
}

void Session::send(pb::NodeMessage& message) {
  message.set_seq(++seq_);
  outgoing_.push_back(message.SerializeAsString());
}

std::vector<std::string> Session::take_outgoing() { return std::move(outgoing_); }

void Session::send_hello(double now) {
  pb::NodeMessage m;
  pb::Hello* h = m.mutable_hello();
  h->set_node_id(config_.node_id);
  h->set_software_version(config_.software_version);
  h->set_protocol_version(kProtocolVersion);
  h->set_profile_id(node_.profile().id);
  h->set_profile_hash(config_.profile_hash);
  h->set_calibration_version(config_.calibration_version);
  h->set_last_plan_generation(node_.plan_generation());
  const auto& caps = node_.capabilities();
  pb::CapabilitySummary* c = h->mutable_capabilities();
  c->set_space(caps.motion.space == capabilities::MotionSpace::Planar ? pb::MOTION_SPACE_PLANAR
                                                                       : pb::MOTION_SPACE_AERIAL);
  c->set_control_interface(capabilities::to_string(caps.control.interface));
  c->set_max_speed_mps(top_speed(caps.motion));
  c->set_safety_ready(caps.safety_ready);
  for (const auto& s : node_.profile().sensors) {
    c->add_sensor_ids(s.id);
  }
  send(m);
  hello_sent_ = now;
}

void Session::on_connected(double now) {
  connected_ = true;
  accepted_ = false;
  if (!rejected_) {
    send_hello(now);
  }
}

void Session::on_disconnected(double) {
  connected_ = false;
  accepted_ = false;
  outgoing_.clear();
}

bool Session::receive(const std::string& bytes, double now) {
  pb::HubMessage m;
  if (!m.ParseFromString(bytes)) {
    return false;
  }
  receive(m, now);
  return true;
}

void Session::receive(const pb::HubMessage& m, double now) {
  if (!connected_) {
    return;
  }
  switch (m.body_case()) {
    case pb::HubMessage::kSessionAccept: {
      const auto& a = m.session_accept();
      accepted_ = true;
      ever_accepted_ = true;
      map_epoch_ = a.map_epoch();
      clock_offset_nanos_ = a.clock_offset_nanos();
      link_.on_session_start(now);
      next_heartbeat_ = next_telemetry_ = now;
      next_evidence_ = now + config_.evidence_period_s;
      return;
    }
    case pb::HubMessage::kSessionReject:
      rejected_ = true;
      reject_detail_ = m.session_reject().detail();
      return;
    default:
      break;
  }
  if (!accepted_) {
    return;
  }
  switch (m.body_case()) {
    case pb::HubMessage::kHeartbeat:
      hub_heartbeat_seq_ = m.heartbeat().seq();
      hub_heartbeat_sent_ = m.heartbeat().sent().unix_nanos();
      link_.on_heartbeat(m.heartbeat().echo_seq(), now);
      outbox_.ack(m.heartbeat().evidence_ack_seq());
      budget_bps_ = static_cast<double>(m.heartbeat().evidence_budget_bytes_per_s());
      break;
    case pb::HubMessage::kMissionPlan:
      handle_plan(m.mission_plan(), now);
      break;
    case pb::HubMessage::kPlanCancel:
      node_.cancel_plan(m.plan_cancel().generation(),
                        m.plan_cancel().then() == pb::PLAN_END_ACTION_RETURN_HOME, now);
      break;
    case pb::HubMessage::kModeRequest:
      switch (m.mode_request().mode()) {
        case pb::MODE_PAUSE:
          node_.set_paused(true, now);
          break;
        case pb::MODE_RESUME:
          node_.set_paused(false, now);
          break;
        case pb::MODE_RETURN_HOME:
          node_.return_home(now);
          break;
        default: {
          pb::Event e;
          e.set_kind(pb::EVENT_KIND_PLAN_REJECTED);
          e.set_detail("mode not supported");
          queue_event(e);
        }
      }
      break;
    case pb::HubMessage::kMapCorrection:
      if (m.map_correction().map_epoch() == map_epoch_) {
        map_to_odom_ = isometry(m.map_correction().map_to_odom());
      }
      break;
    default:
      // Stream requests and map deltas are not used by Argo yet.
      break;
  }
}

void Session::handle_plan(const pb::MissionPlan& p, double now) {
  if (p.map_epoch() != map_epoch_) {
    pb::Event e;
    e.set_kind(pb::EVENT_KIND_PLAN_REJECTED);
    e.set_plan_generation(p.generation());
    e.set_detail("plan is for map epoch " + std::to_string(p.map_epoch()) + ", not " +
                 std::to_string(map_epoch_));
    queue_event(e);
    return;
  }
  Plan plan;
  plan.generation = p.generation();
  plan.expires = hub_to_node(p.expires().unix_nanos());
  for (const auto& w : p.waypoints()) {
    Waypoint wp;
    wp.id = w.id();
    wp.position = map_to_odom_ * vec(w.pose().position());
    if (w.position_tolerance_m() > 0.0) {
      wp.tolerance_m = w.position_tolerance_m();
    }
    wp.speed_cap_mps = w.speed_cap_mps();
    plan.waypoints.push_back(wp);
  }
  node_.set_plan(plan, now);
}

void Session::queue_event(pb::Event e) {
  events_.push_back(std::move(e));
  while (events_.size() > config_.max_buffered_events) {
    events_.pop_front();
  }
}

pb::Event Session::node_event(const Event& e) const {
  pb::Event out;
  set_stamp(out.mutable_stamp(), e.stamp);
  out.set_plan_generation(node_.plan_generation());
  out.set_waypoint_id(e.waypoint_id);
  out.set_detail(e.detail);
  switch (e.kind) {
    case EventKind::WaypointReached:
      out.set_kind(pb::EVENT_KIND_STEP_REACHED);
      break;
    case EventKind::Blocked:
      out.set_kind(pb::EVENT_KIND_STEP_FAILED);
      out.set_cause(to_proto(e.cause));
      break;
    case EventKind::PlanRejected:
      out.set_kind(pb::EVENT_KIND_PLAN_REJECTED);
      break;
    case EventKind::PlanExpired:
      out.set_kind(pb::EVENT_KIND_PLAN_REJECTED);
      out.set_detail("plan expired");
      break;
    case EventKind::FailsafeEntered:
      out.set_kind(pb::EVENT_KIND_FAILSAFE_ENTERED);
      out.set_cause(pb::FAILURE_CAUSE_LINK_LOST);
      break;
    case EventKind::FailsafeDone:
      out.set_kind(pb::EVENT_KIND_FAILSAFE_DONE);
      break;
    case EventKind::FailsafeCleared:
      out.set_kind(pb::EVENT_KIND_FAILSAFE_CLEARED);
      break;
    case EventKind::PlanAccepted:
    case EventKind::PlanComplete:
      out.set_kind(pb::EVENT_KIND_UNSPECIFIED);
      break;
  }
  return out;
}

void Session::on_link_change(LinkState from, LinkState to, double now) {
  pb::Event e;
  set_stamp(e.mutable_stamp(), now);
  e.set_kind(pb::EVENT_KIND_LINK_STATE_CHANGED);
  e.set_plan_generation(node_.plan_generation());
  e.set_detail(to_string(from) + " -> " + to_string(to));
  queue_event(e);

  if (to == LinkState::Lost) {
    node_.set_speed_cap(0.0);
    if (ever_accepted_) {
      node_.enter_failsafe(node_.profile().failsafe.on_link_lost, now);
    }
    return;
  }
  if (from == LinkState::Lost) {
    node_.clear_failsafe(now);
  }
  node_.set_speed_cap(to == LinkState::Degraded
                          ? config_.degraded_speed_fraction * top_speed(node_.capabilities().motion)
                          : 0.0);
}

TickResult Session::step(double now) {
  const LinkState before = link_.state();
  const LinkState after = link_.update(now);
  if (after != before) {
    on_link_change(before, after, now);
  }

  TickResult tick = node_.tick(now);
  for (const Event& e : tick.events) {
    pb::Event out = node_event(e);
    if (out.kind() != pb::EVENT_KIND_UNSPECIFIED) {
      queue_event(std::move(out));
    }
  }

  // Evidence is kept whatever the link does, merged per cell, and goes out once it can.
  outbox_.add(node_.take_evidence());

  if (!connected_ || !accepted_) {
    if (connected_ && !rejected_ && now - hello_sent_ >= config_.hello_retry_s) {
      send_hello(now);
    }
    return tick;
  }

  while (!events_.empty()) {
    pb::NodeMessage m;
    *m.mutable_event() = std::move(events_.front());
    events_.pop_front();
    send(m);
  }
  if (now >= next_heartbeat_) {
    next_heartbeat_ = now + config_.heartbeat_period_s;
    pb::NodeMessage m;
    pb::Heartbeat* hb = m.mutable_heartbeat();
    hb->set_seq(++heartbeat_seq_);
    set_stamp(hb->mutable_sent(), now);
    hb->set_echo_seq(hub_heartbeat_seq_);
    hb->mutable_echo_sent()->set_unix_nanos(hub_heartbeat_sent_);
    hb->set_evidence_pending_bytes(outbox_.backlog_bytes());
    link_.on_sent(heartbeat_seq_, now);
    send(m);
  }
  if (now >= next_telemetry_ && node_.has_odometry()) {
    next_telemetry_ = now + config_.telemetry_period_s;
    const Odometry& o = node_.odometry();
    pb::NodeMessage m;
    pb::Telemetry* t = m.mutable_telemetry();
    set_stamp(t->mutable_stamp(), o.stamp);
    const Eigen::Isometry3d odom_pose = o.pose.transform();
    set_pose(t->mutable_pose_odom(), odom_pose);
    set_pose(t->mutable_pose_map(), map_to_odom_.inverse() * odom_pose);
    t->set_map_epoch(map_epoch_);
    set_vec(t->mutable_velocity()->mutable_linear_mps(), o.pose.rotate(o.twist.linear));
    t->mutable_velocity()->mutable_angular_radps()->set_z(o.twist.yaw_rate);
    t->mutable_link()->set_state(to_proto(link_.state()));
    t->mutable_link()->set_round_trip_ms(link_.round_trip_s() * 1e3);
    t->mutable_link()->set_loss_fraction(link_.loss_fraction());
    pb::SafetyState safety = pb::SAFETY_STATE_NOMINAL;
    if (node_.failsafe_active()) {
      safety = pb::SAFETY_STATE_FAILSAFE;
    } else if (!tick.target_id.empty() && tick.command.stop) {
      safety = pb::SAFETY_STATE_STOPPED;
    } else if (node_.speed_cap() > 0.0) {
      safety = pb::SAFETY_STATE_LIMITED;
    }
    t->set_safety(safety);
    t->set_plan_generation(node_.plan_generation());
    t->set_active_waypoint_id(tick.target_id);
    send(m);
  }
  if (now >= next_evidence_) {
    next_evidence_ = now + config_.evidence_period_s;
    send_evidence(now);
  }
  return tick;
}

void Session::send_evidence(double now) {
  // While the link is lost, messages would only be lost with it; they wait instead.
  if (link_.state() == LinkState::Lost) {
    return;
  }
  // The hub's budget, as a token bucket a little under it, so the hub never has to drop.
  size_t allowance = config_.max_evidence_message_bytes;
  if (budget_bps_ > 0.0) {
    const double rate = 0.9 * budget_bps_;
    const double burst = std::max(2048.0, 2.0 * config_.evidence_period_s * rate);
    budget_tokens_ = budget_at_ < 0.0 ? burst : std::min(burst, budget_tokens_ + (now - budget_at_) * rate);
    budget_at_ = now;
    allowance = std::min(allowance, static_cast<size_t>(std::max(0.0, budget_tokens_)));
  }
  size_t used = 0;
  // Unacknowledged messages first, oldest first, after a timeout well over the round trip.
  const double timeout = std::max(1.0, 4.0 * link_.round_trip_s());
  for (const EvidenceOutbox::Batch& b : outbox_.due(now, timeout, allowance)) {
    send_evidence_message(b, now);
    used += b.payload.size();
  }
  if (auto b = outbox_.next(allowance - std::min(allowance, used), now)) {
    send_evidence_message(*b, now);
    used += b->payload.size();
  }
  if (budget_bps_ > 0.0) {
    budget_tokens_ -= static_cast<double>(used);
  }
}

void Session::send_evidence_message(const EvidenceOutbox::Batch& batch, double now) {
  pb::NodeMessage m;
  pb::LocalEvidence* ev = m.mutable_local_evidence();
  set_stamp(ev->mutable_stamp(), now);
  ev->set_map_epoch(map_epoch_);
  set_pose(ev->mutable_map_to_odom(), map_to_odom_);
  ev->set_resolution_m(node_.map().resolution());
  ev->set_encoding(pb::EVIDENCE_ENCODING_VOXEL_DELTA);
  ev->set_payload(batch.payload);
  ev->set_seq(batch.seq);
  ev->set_stream(outbox_.stream());
  evidence_bytes_ += ev->payload().size();
  send(m);
}

}  // namespace argus::node

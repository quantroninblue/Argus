#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <map>

#include "argus/node/grpc_link.hpp"
#include "argus/node/node.hpp"
#include "argus/node/session.hpp"

namespace py = pybind11;
using namespace argus::node;

namespace {

Vec3 vec3(const py::sequence& s) {
  if (py::len(s) != 3) {
    throw py::value_error("expected three values");
  }
  return Vec3(s[0].cast<double>(), s[1].cast<double>(), s[2].cast<double>());
}

py::dict event_dict(const Event& e) {
  py::dict d;
  d["stamp"] = e.stamp;
  d["kind"] = to_string(e.kind);
  d["waypoint_id"] = e.waypoint_id;
  d["cause"] = to_string(e.cause);
  d["detail"] = e.detail;
  return d;
}

py::dict twist_dict(const Twist& t) {
  py::dict d;
  d["linear"] = t.linear;
  d["yaw_rate"] = t.yaw_rate;
  return d;
}

py::dict tick_dict(const TickResult& r) {
  py::dict c;
  c["stamp"] = r.command.stamp;
  c["stop"] = r.command.stop;
  c["velocity"] = twist_dict(r.command.velocity);
  c["setpoint_position"] = r.command.setpoint.position;
  c["setpoint_yaw"] = r.command.setpoint.yaw;
  c["setpoint_velocity"] = r.command.setpoint_velocity;
  c["setpoint_yaw_rate"] = r.command.setpoint_yaw_rate;
  py::dict d;
  d["command"] = c;
  d["flags"] = flag_names(r.safety_flags);
  d["proposed"] = twist_dict(r.proposed);
  d["target_id"] = r.target_id;
  py::dict planner;
  planner["path_reaches_target"] = r.proposal.path_reaches_target;
  planner["candidates"] = r.proposal.candidates;
  planner["clear_rollouts"] = r.proposal.clear_rollouts;
  planner["off_route"] = r.proposal.off_route;
  planner["target_blocked"] = r.proposal.target_blocked;
  planner["admission_checks"] = r.proposal.admission_checks;
  d["planner"] = planner;
  py::list events;
  for (const Event& e : r.events) {
    events.append(event_dict(e));
  }
  d["events"] = events;
  return d;
}

std::string cell_name(Cell c) {
  switch (c) {
    case Cell::Free:
      return "free";
    case Cell::Occupied:
      return "occupied";
    case Cell::Unknown:
      break;
  }
  return "unknown";
}

}  // namespace

PYBIND11_MODULE(node, m) {
  m.doc() = "Argo, the Argus node: local map, planner, safety supervisor and command adapters.";

  py::class_<LocalMapConfig>(m, "LocalMapConfig")
      .def(py::init<>())
      .def_readwrite("resolution_m", &LocalMapConfig::resolution_m)
      .def_readwrite("half_width_m", &LocalMapConfig::half_width_m)
      .def_readwrite("z_below_m", &LocalMapConfig::z_below_m)
      .def_readwrite("z_above_m", &LocalMapConfig::z_above_m)
      .def_readwrite("record_evidence", &LocalMapConfig::record_evidence);

  py::class_<RayPatternOptions>(m, "RayPatternOptions")
      .def(py::init<>())
      .def_readwrite("range_2d_step_deg", &RayPatternOptions::range_2d_step_deg)
      .def_readwrite("range_3d_h_step_deg", &RayPatternOptions::range_3d_h_step_deg)
      .def_readwrite("range_3d_v_step_deg", &RayPatternOptions::range_3d_v_step_deg)
      .def_readwrite("depth_cols", &RayPatternOptions::depth_cols)
      .def_readwrite("depth_rows", &RayPatternOptions::depth_rows);

  py::class_<PlannerConfig>(m, "PlannerConfig")
      .def(py::init<>())
      .def_readwrite("period_s", &PlannerConfig::period_s)
      .def_readwrite("replan_period_s", &PlannerConfig::replan_period_s)
      .def_readwrite("horizon_s", &PlannerConfig::horizon_s)
      .def_readwrite("rollout_step_s", &PlannerConfig::rollout_step_s)
      .def_readwrite("unknown_penalty", &PlannerConfig::unknown_penalty)
      .def_readwrite("max_admission_checks", &PlannerConfig::max_admission_checks)
      .def_readwrite("people_cost", &PlannerConfig::people_cost)
      .def_readwrite("watch_movers_m", &PlannerConfig::watch_movers_m)
      .def_readwrite("corridor_m", &PlannerConfig::corridor_m)
      .def_readwrite("people_sigma_m", &PlannerConfig::people_sigma_m)
      .def_readwrite("people_ahead_s", &PlannerConfig::people_ahead_s);

  py::class_<SafetyConfig>(m, "SafetyConfig")
      .def(py::init<>())
      .def_readwrite("command_lease_s", &SafetyConfig::command_lease_s)
      .def_readwrite("odometry_max_age_s", &SafetyConfig::odometry_max_age_s)
      .def_readwrite("max_unknown_fraction", &SafetyConfig::max_unknown_fraction)
      .def_readwrite("envelope_spacing_m", &SafetyConfig::envelope_spacing_m)
      .def_readwrite("mover_margin_m", &SafetyConfig::mover_margin_m)
      .def_readwrite("mover_sigmas", &SafetyConfig::mover_sigmas)
      .def_readwrite("mover_widen_max_m", &SafetyConfig::mover_widen_max_m)
      .def_readwrite("people_near_m", &SafetyConfig::people_near_m)
      .def_readwrite("people_far_m", &SafetyConfig::people_far_m)
      .def_readwrite("people_speed_mps", &SafetyConfig::people_speed_mps);

  py::class_<NodeConfig>(m, "NodeConfig")
      .def(py::init<>())
      .def_readwrite("map", &NodeConfig::map)
      .def_readwrite("rays", &NodeConfig::rays)
      .def_readwrite("planner", &NodeConfig::planner)
      .def_readwrite("safety", &NodeConfig::safety)
      .def_readwrite("min_obstacle_height_m", &NodeConfig::min_obstacle_height_m)
      .def_readwrite("drop_depth_m", &NodeConfig::drop_depth_m)
      .def_readwrite("track_movers", &NodeConfig::track_movers)
      .def_readwrite("check_odometry", &NodeConfig::check_odometry)
      .def_readwrite("yield_wait_s", &NodeConfig::yield_wait_s)
      .def_readwrite("static_after_s", &NodeConfig::static_after_s)
      .def_readwrite("no_return_is_free", &NodeConfig::no_return_is_free)
      .def_readwrite("no_progress_s", &NodeConfig::no_progress_s)
      .def_readwrite("stuck_report_s", &NodeConfig::stuck_report_s)
      .def_readwrite("dead_end_s", &NodeConfig::dead_end_s);

  py::class_<LinkConfig>(m, "LinkConfig")
      .def(py::init<>())
      .def_readwrite("degraded_rtt_s", &LinkConfig::degraded_rtt_s)
      .def_readwrite("degraded_loss", &LinkConfig::degraded_loss)
      .def_readwrite("silent_heartbeats", &LinkConfig::silent_heartbeats)
      .def_readwrite("recover_after_s", &LinkConfig::recover_after_s);

  py::class_<SessionConfig>(m, "SessionConfig")
      .def(py::init<>())
      .def_readwrite("node_id", &SessionConfig::node_id)
      .def_readwrite("software_version", &SessionConfig::software_version)
      .def_readwrite("profile_hash", &SessionConfig::profile_hash)
      .def_readwrite("calibration_version", &SessionConfig::calibration_version)
      .def_readwrite("heartbeat_period_s", &SessionConfig::heartbeat_period_s)
      .def_readwrite("telemetry_period_s", &SessionConfig::telemetry_period_s)
      .def_readwrite("evidence_period_s", &SessionConfig::evidence_period_s)
      .def_readwrite("degraded_speed_fraction", &SessionConfig::degraded_speed_fraction)
      .def_readwrite("link", &SessionConfig::link);

  py::class_<Node>(m, "Node")
      .def(py::init([](const std::string& profile_text, const NodeConfig& config) {
             return std::make_unique<Node>(argus::capabilities::load_profile_text(profile_text), config);
           }),
           py::arg("profile_text"), py::arg("config") = NodeConfig())
      .def(
          "on_odometry",
          [](Node& n, double stamp, const py::sequence& position, double yaw, const py::sequence& linear,
             double yaw_rate, bool healthy, double roll, double pitch) {
            Odometry o;
            o.stamp = stamp;
            o.pose.position = vec3(position);
            o.pose.yaw = yaw;
            o.twist.linear = vec3(linear);
            o.twist.yaw_rate = yaw_rate;
            o.healthy = healthy;
            o.roll = roll;
            o.pitch = pitch;
            n.on_odometry(o);
          },
          py::arg("stamp"), py::arg("position"), py::arg("yaw"), py::arg("linear"), py::arg("yaw_rate"),
          py::arg("healthy") = true, py::arg("roll") = 0.0, py::arg("pitch") = 0.0)
      .def(
          "on_range_frame",
          [](Node& n, double stamp, const std::string& sensor_id,
             py::array_t<float, py::array::c_style | py::array::forcecast> ranges) {
            RangeFrame f;
            f.stamp = stamp;
            f.sensor_id = sensor_id;
            f.ranges.assign(ranges.data(), ranges.data() + ranges.size());
            return n.on_range_frame(f);
          },
          py::arg("stamp"), py::arg("sensor_id"), py::arg("ranges"))
      .def(
          "set_plan",
          [](Node& n, uint64_t generation, double expires, const py::list& waypoints, double now) {
            Plan p;
            p.generation = generation;
            p.expires = expires;
            for (const auto& item : waypoints) {
              const py::dict w = item.cast<py::dict>();
              Waypoint wp;
              wp.id = w["id"].cast<std::string>();
              wp.position = vec3(w["position"].cast<py::sequence>());
              if (w.contains("tolerance_m")) wp.tolerance_m = w["tolerance_m"].cast<double>();
              if (w.contains("speed_cap_mps")) wp.speed_cap_mps = w["speed_cap_mps"].cast<double>();
              p.waypoints.push_back(wp);
            }
            return n.set_plan(p, now);
          },
          py::arg("generation"), py::arg("expires"), py::arg("waypoints"), py::arg("now"))
      .def("tick", [](Node& n, double now) { return tick_dict(n.tick(now)); }, py::arg("now"))
      .def("range_sensor_ids", &Node::range_sensor_ids)
      .def(
          "ray_directions",
          [](const Node& n, const std::string& id) {
            const RayPattern& p = n.ray_pattern(id);
            py::array_t<double> out({static_cast<py::ssize_t>(p.size()), py::ssize_t{3}});
            auto view = out.mutable_unchecked<2>();
            for (size_t i = 0; i < p.size(); ++i) {
              for (int a = 0; a < 3; ++a) view(i, a) = p.directions[i][a];
            }
            return out;
          },
          py::arg("sensor_id"))
      .def(
          "self_hits",
          [](const Node& n, const std::string& id) {
            const auto& h = n.self_hits(id);
            return py::array_t<double>(static_cast<py::ssize_t>(h.size()), h.data());
          },
          py::arg("sensor_id"))
      .def(
          "sensor_mount", [](const Node& n, const std::string& id) { return Eigen::Matrix4d(n.sensor_mount(id).matrix()); },
          py::arg("sensor_id"))
      .def(
          "cell", [](const Node& n, const py::sequence& p) { return cell_name(n.map().at(vec3(p))); },
          py::arg("position"))
      .def(
          "ever_free", [](const Node& n, const py::sequence& p) { return n.map().ever_free(n.map().index_of(vec3(p))); },
          py::arg("position"))
      .def("path", &Node::path)
      .def("movers",
           [](const Node& n) {
             py::list out;
             for (const TrackedObstacle& o : n.movers()) {
               py::dict d;
               d["id"] = o.id;
               d["position"] = py::make_tuple(o.position.x(), o.position.y());
               d["velocity"] = py::make_tuple(o.velocity.x(), o.velocity.y());
               d["radius_m"] = o.radius_m;
               d["z_span"] = py::make_tuple(o.z_low_m, o.z_high_m);
               d["moving"] = o.moving;
               d["has_moved"] = o.has_moved;
               d["remembered"] = o.remembered;
               d["position_sigma_m"] = o.position_sigma_m;
               d["velocity_sigma_mps"] = o.velocity_sigma_mps;
               d["confirmed"] = o.confirmed;
               out.append(d);
             }
             return out;
           })
      .def_property_readonly("control_period_s", &Node::control_period_s)
      .def_property_readonly("command_lease_s", &Node::command_lease_s)
      .def_property_readonly("profile_id", [](const Node& n) { return n.profile().id; })
      .def_property_readonly("control_interface", [](const Node& n) {
        return argus::capabilities::to_string(n.capabilities().control.interface);
      })
      .def_property_readonly("plan_generation", &Node::plan_generation)
      .def_property_readonly("failsafe_active", &Node::failsafe_active)
      .def_property_readonly("speed_cap", &Node::speed_cap)
      .def("set_speed_cap", &Node::set_speed_cap, py::arg("mps"))
      .def(
          "enter_failsafe",
          [](Node& n, const std::string& action, double now) {
            using argus::capabilities::LinkLostAction;
            static const std::map<std::string, LinkLostAction> actions = {
                {"stop", LinkLostAction::Stop},
                {"hold", LinkLostAction::Hold},
                {"land", LinkLostAction::Land}};
            const auto it = actions.find(action);
            if (it == actions.end()) throw py::value_error("unknown failsafe action '" + action + "'");
            n.enter_failsafe(it->second, now);
          },
          py::arg("action"), py::arg("now"))
      .def("clear_failsafe", &Node::clear_failsafe, py::arg("now"))
      .def("return_home", &Node::return_home, py::arg("now"))
      .def("set_paused", &Node::set_paused, py::arg("paused"), py::arg("now"))
      .def("breadcrumbs", [](const Node& n) { return n.breadcrumbs(); })
      .def("sensor_faults", [](const Node& n) { return n.sensor_faults(); })
      .def("odometry_check", [](const Node& n) {
        const auto& c = n.odometry_check();
        py::dict d;
        d["suspect"] = c.suspect;
        d["shift_m"] = c.shift_m;
        d["turn_rad"] = c.turn_rad;
        d["verdicts"] = c.verdicts;
        d["checks"] = c.checks;
        d["last_zero"] = c.last_zero;
        d["last_best"] = c.last_best;
        d["last_points"] = c.last_points;
        return d;
      });

  py::class_<GrpcLinkConfig>(m, "GrpcLinkConfig")
      .def(py::init<>())
      .def_readwrite("target", &GrpcLinkConfig::target)
      .def_readwrite("ca_pem", &GrpcLinkConfig::ca_pem)
      .def_readwrite("cert_pem", &GrpcLinkConfig::cert_pem)
      .def_readwrite("key_pem", &GrpcLinkConfig::key_pem)
      .def_readwrite("server_name", &GrpcLinkConfig::server_name)
      .def_readwrite("connect_timeout_s", &GrpcLinkConfig::connect_timeout_s)
      .def_readwrite("reconnect_period_s", &GrpcLinkConfig::reconnect_period_s);

  // poll() returns {"transitions": [(connected, after), ...], "received": [bytes, ...]}, where
  // `after` counts the received messages that came before that change of state.
  py::class_<GrpcLink>(m, "GrpcLink")
      .def(py::init<GrpcLinkConfig>(), py::arg("config"), py::call_guard<py::gil_scoped_release>())
      .def(
          "send",
          [](GrpcLink& link, const py::list& messages) {
            std::vector<std::string> out;
            for (const auto& b : messages) out.push_back(b.cast<std::string>());
            py::gil_scoped_release release;
            link.send(out);
          },
          py::arg("messages"))
      .def("poll",
           [](GrpcLink& link) {
             GrpcLink::Update u;
             {
               py::gil_scoped_release release;
               u = link.poll();
             }
             py::list received;
             for (const std::string& b : u.received) received.append(py::bytes(b));
             py::list transitions;
             for (const auto& t : u.transitions) transitions.append(py::make_tuple(t.connected, t.after));
             py::dict d;
             d["transitions"] = transitions;
             d["received"] = received;
             return d;
           })
      .def_property_readonly("connected", &GrpcLink::connected)
      .def_property_readonly("streams_opened", &GrpcLink::streams_opened);

  py::class_<Session>(m, "Session")
      .def(py::init<Node&, SessionConfig>(), py::arg("node"), py::arg("config"), py::keep_alive<1, 2>())
      .def("on_connected", &Session::on_connected, py::arg("now"))
      .def("on_disconnected", &Session::on_disconnected, py::arg("now"))
      .def(
          "receive",
          [](Session& s, const py::bytes& data, double now) { return s.receive(std::string(data), now); },
          py::arg("data"), py::arg("now"))
      .def(
          "step", [](Session& s, double now) { return tick_dict(s.step(now)); }, py::arg("now"))
      .def("take_outgoing",
           [](Session& s) {
             py::list out;
             for (const std::string& b : s.take_outgoing()) out.append(py::bytes(b));
             return out;
           })
      .def_property_readonly("connected", &Session::connected)
      .def_property_readonly("accepted", &Session::accepted)
      .def_property_readonly("rejected", &Session::rejected)
      .def_property_readonly("reject_detail", &Session::reject_detail)
      .def_property_readonly("link_state", [](const Session& s) { return to_string(s.link_state()); })
      .def_property_readonly("round_trip_s", [](const Session& s) { return s.link().round_trip_s(); })
      .def_property_readonly("map_epoch", &Session::map_epoch)
      .def_property_readonly("evidence_bytes", &Session::evidence_bytes)
      .def_property_readonly("evidence_backlog_bytes", [](const Session& s) { return s.evidence().backlog_bytes(); })
      .def_property_readonly("evidence_unacked", [](const Session& s) { return s.evidence().unacked(); })
      .def_property_readonly("evidence_budget", &Session::evidence_budget);
}

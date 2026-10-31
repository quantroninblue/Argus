#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "argus/node/node.hpp"

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
      .def_readwrite("z_above_m", &LocalMapConfig::z_above_m);

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
      .def_readwrite("max_admission_checks", &PlannerConfig::max_admission_checks);

  py::class_<SafetyConfig>(m, "SafetyConfig")
      .def(py::init<>())
      .def_readwrite("command_lease_s", &SafetyConfig::command_lease_s)
      .def_readwrite("odometry_max_age_s", &SafetyConfig::odometry_max_age_s)
      .def_readwrite("max_unknown_fraction", &SafetyConfig::max_unknown_fraction)
      .def_readwrite("envelope_spacing_m", &SafetyConfig::envelope_spacing_m);

  py::class_<NodeConfig>(m, "NodeConfig")
      .def(py::init<>())
      .def_readwrite("map", &NodeConfig::map)
      .def_readwrite("rays", &NodeConfig::rays)
      .def_readwrite("planner", &NodeConfig::planner)
      .def_readwrite("safety", &NodeConfig::safety)
      .def_readwrite("min_obstacle_height_m", &NodeConfig::min_obstacle_height_m)
      .def_readwrite("no_return_is_free", &NodeConfig::no_return_is_free)
      .def_readwrite("no_progress_s", &NodeConfig::no_progress_s);

  py::class_<Node>(m, "Node")
      .def(py::init([](const std::string& profile_text, const NodeConfig& config) {
             return std::make_unique<Node>(argus::capabilities::load_profile_text(profile_text), config);
           }),
           py::arg("profile_text"), py::arg("config") = NodeConfig())
      .def(
          "on_odometry",
          [](Node& n, double stamp, const py::sequence& position, double yaw, const py::sequence& linear,
             double yaw_rate, bool healthy) {
            Odometry o;
            o.stamp = stamp;
            o.pose.position = vec3(position);
            o.pose.yaw = yaw;
            o.twist.linear = vec3(linear);
            o.twist.yaw_rate = yaw_rate;
            o.healthy = healthy;
            n.on_odometry(o);
          },
          py::arg("stamp"), py::arg("position"), py::arg("yaw"), py::arg("linear"), py::arg("yaw_rate"),
          py::arg("healthy") = true)
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
      .def("path", &Node::path)
      .def_property_readonly("control_period_s", &Node::control_period_s)
      .def_property_readonly("command_lease_s", &Node::command_lease_s)
      .def_property_readonly("profile_id", [](const Node& n) { return n.profile().id; })
      .def_property_readonly("control_interface", [](const Node& n) {
        return argus::capabilities::to_string(n.capabilities().control.interface);
      });
}

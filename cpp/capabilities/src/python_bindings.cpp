#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "argus/capabilities/capabilities.hpp"

namespace py = pybind11;
using namespace argus::capabilities;

namespace {

Axis parse_axis(const std::string& name) {
  if (name == "x") return Axis::X;
  if (name == "y") return Axis::Y;
  if (name == "z") return Axis::Z;
  throw py::value_error("axis must be 'x', 'y' or 'z'");
}

py::dict control_dict(const ControlSpec& c) {
  py::dict d;
  d["interface"] = to_string(c.interface);
  d["transport"] = to_string(c.transport);
  d["endpoint"] = c.endpoint;
  d["rate_hz"] = c.rate_hz;
  d["latency_s"] = c.latency_s;
  d["controller_timeout_s"] = c.controller_timeout_s;
  return d;
}

py::dict failsafe_dict(const FailsafeSpec& f) {
  py::dict d;
  d["link_lost_after_s"] = f.link_lost_after_s;
  d["on_link_lost"] = to_string(f.on_link_lost);
  d["uncovered_motion"] = to_string(f.uncovered_motion);
  d["creep_speed_mps"] = f.creep_speed_mps;
  return d;
}

}  // namespace

PYBIND11_MODULE(capabilities, m) {
  m.doc() = "Turns Argus body profiles into the capability model the navigation core reads.";

  py::register_exception<ProfileError>(m, "ProfileError", PyExc_ValueError);

  py::class_<DeriveOptions>(m, "DeriveOptions")
      .def(py::init<>())
      .def_readwrite("azimuth_step_deg", &DeriveOptions::azimuth_step_deg)
      .def_readwrite("elevation_step_deg", &DeriveOptions::elevation_step_deg)
      .def_readwrite("evaluation_range_m", &DeriveOptions::evaluation_range_m)
      .def_readwrite("sample_step_m", &DeriveOptions::sample_step_m)
      .def_readwrite("band_height_m", &DeriveOptions::band_height_m)
      .def_readwrite("min_obstacle_height_m", &DeriveOptions::min_obstacle_height_m)
      .def_readwrite("max_near_gap_m", &DeriveOptions::max_near_gap_m);

  py::class_<AxisCapability>(m, "AxisCapability")
      .def_readonly("min_mps", &AxisCapability::min_mps)
      .def_readonly("max_mps", &AxisCapability::max_mps)
      .def_readonly("accel_mps2", &AxisCapability::accel_mps2)
      .def_readonly("decel_mps2", &AxisCapability::decel_mps2)
      .def_property_readonly("enabled", &AxisCapability::enabled);

  py::class_<MotionCapability>(m, "MotionCapability")
      .def_property_readonly("space", [](const MotionCapability& c) { return to_string(c.space); })
      .def_readonly("can_reverse", &MotionCapability::can_reverse)
      .def_readonly("can_strafe", &MotionCapability::can_strafe)
      .def_readonly("can_turn_in_place", &MotionCapability::can_turn_in_place)
      .def_readonly("can_hover", &MotionCapability::can_hover)
      .def_readonly("min_turn_radius_m", &MotionCapability::min_turn_radius_m)
      .def_readonly("x", &MotionCapability::x)
      .def_readonly("y", &MotionCapability::y)
      .def_readonly("z", &MotionCapability::z)
      .def_readonly("yaw_rate_max_radps", &MotionCapability::yaw_rate_max_radps)
      .def_readonly("yaw_accel_radps2", &MotionCapability::yaw_accel_radps2)
      .def_readonly("latency_s", &MotionCapability::latency_s)
      .def(
          "stopping_distance_m",
          [](const MotionCapability& c, const std::string& axis, double speed) {
            return c.stopping_distance_m(parse_axis(axis), speed);
          },
          py::arg("axis"), py::arg("speed_mps"));

  py::class_<CollisionCapability>(m, "CollisionCapability")
      .def_readonly("aabb_min", &CollisionCapability::aabb_min)
      .def_readonly("aabb_max", &CollisionCapability::aabb_max)
      .def_readonly("footprint", &CollisionCapability::footprint)
      .def_readonly("bounding_radius_m", &CollisionCapability::bounding_radius_m)
      .def_readonly("body_footprint", &CollisionCapability::body_footprint)
      .def_readonly("body_radius_m", &CollisionCapability::body_radius_m)
      .def_readonly("inscribed_radius_m", &CollisionCapability::inscribed_radius_m)
      .def_readonly("padding_m", &CollisionCapability::padding_m)
      .def("edge_distance_m", &CollisionCapability::edge_distance_m, py::arg("azimuth_rad"),
           py::arg("elevation_rad") = 0.0);

  py::class_<HeightBand>(m, "HeightBand")
      .def_readonly("low_m", &HeightBand::low_m)
      .def_readonly("high_m", &HeightBand::high_m);

  py::class_<CoverageBin>(m, "CoverageBin")
      .def_readonly("azimuth_deg", &CoverageBin::azimuth_deg)
      .def_readonly("elevation_deg", &CoverageBin::elevation_deg)
      .def_readonly("edge_distance_m", &CoverageBin::edge_distance_m)
      .def_readonly("first_seen_m", &CoverageBin::first_seen_m)
      .def_readonly("max_seen_m", &CoverageBin::max_seen_m)
      .def_readonly("full_height_from_m", &CoverageBin::full_height_from_m)
      .def_readonly("full_height_reach_m", &CoverageBin::full_height_reach_m)
      .def_readonly("unseen_bands", &CoverageBin::unseen_bands)
      .def_readonly("sensors", &CoverageBin::sensors)
      .def_property_readonly("seen", &CoverageBin::seen);

  py::class_<SensingCapability>(m, "SensingCapability")
      .def_readonly("evaluation_range_m", &SensingCapability::evaluation_range_m)
      .def_readonly("sample_step_m", &SensingCapability::sample_step_m)
      .def_readonly("bands", &SensingCapability::bands)
      .def_readonly("bins", &SensingCapability::bins)
      .def_property_readonly("seen_fraction", &SensingCapability::seen_fraction)
      .def_property_readonly("full_height_fraction", &SensingCapability::full_height_fraction);

  py::class_<LocalizationCapability>(m, "LocalizationCapability")
      .def_readonly("inertial", &LocalizationCapability::inertial)
      .def_readonly("wheel_odometry", &LocalizationCapability::wheel_odometry)
      .def_readonly("external_odometry", &LocalizationCapability::external_odometry)
      .def_readonly("visual", &LocalizationCapability::visual)
      .def_readonly("range", &LocalizationCapability::range)
      .def_property_readonly("sources", &LocalizationCapability::sources);

  py::class_<ComputeCapability>(m, "ComputeCapability")
      .def_readonly("board", &ComputeCapability::board)
      .def_property_readonly("accelerator",
                             [](const ComputeCapability& c) { return to_string(c.accelerator); })
      .def_readonly("inference_backends", &ComputeCapability::inference_backends);

  py::class_<Capabilities>(m, "Capabilities")
      .def_readonly("profile_id", &Capabilities::profile_id)
      .def_readonly("motion", &Capabilities::motion)
      .def_readonly("collision", &Capabilities::collision)
      .def_readonly("sensing", &Capabilities::sensing)
      .def_readonly("localization", &Capabilities::localization)
      .def_readonly("compute", &Capabilities::compute)
      .def_property_readonly("control", [](const Capabilities& c) { return control_dict(c.control); })
      .def_property_readonly("failsafe",
                             [](const Capabilities& c) { return failsafe_dict(c.failsafe); })
      .def_readonly("min_obstacle_height_m", &Capabilities::min_obstacle_height_m)
      .def_readonly("assumed_obstacle_height_m", &Capabilities::assumed_obstacle_height_m)
      .def_readonly("assumed_clear_at_start_m", &Capabilities::assumed_clear_at_start_m)
      .def_readonly("safety_ready", &Capabilities::safety_ready)
      .def_readonly("sees_above", &Capabilities::sees_above)
      .def_readonly("max_flight_height_m", &Capabilities::max_flight_height_m)
      .def_readonly("missing_fields", &Capabilities::missing_fields)
      .def_readonly("unreviewed_fields", &Capabilities::unreviewed_fields);

  m.def("validate_profile_text", &validate_profile_text, py::arg("text"),
        "Returns every problem found in a profile; empty when it is valid.");
  m.def(
      "derive_from_text",
      [](const std::string& text, const DeriveOptions& options) {
        return derive_capabilities(load_profile_text(text), options);
      },
      py::arg("text"), py::arg("options") = DeriveOptions());
  m.def(
      "derive_from_file",
      [](const std::string& path, const DeriveOptions& options) {
        return derive_capabilities(load_profile_file(path), options);
      },
      py::arg("path"), py::arg("options") = DeriveOptions());
}

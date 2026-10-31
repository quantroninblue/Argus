#include "argus/capabilities/profile.hpp"

#include <cmath>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace argus::capabilities {

namespace {

std::string join_problems(const std::vector<std::string>& problems) {
  std::ostringstream out;
  out << "invalid profile";
  for (const auto& p : problems) {
    out << "\n  " << p;
  }
  return out.str();
}

template <typename Enum>
struct EnumTable {
  std::vector<std::pair<const char*, Enum>> entries;

  std::optional<Enum> parse(const std::string& text) const {
    for (const auto& [name, value] : entries) {
      if (text == name) {
        return value;
      }
    }
    return std::nullopt;
  }

  std::string name(Enum value) const {
    for (const auto& [name, v] : entries) {
      if (v == value) {
        return name;
      }
    }
    return "unknown";
  }

  std::string choices() const {
    std::string out;
    for (const auto& [name, v] : entries) {
      out += out.empty() ? "" : ", ";
      out += name;
    }
    return out;
  }
};

const EnumTable<MotionSpace> kMotionSpaces{{{"planar", MotionSpace::Planar},
                                            {"aerial", MotionSpace::Aerial}}};
const EnumTable<ControlInterface> kInterfaces{{{"velocity", ControlInterface::Velocity},
                                               {"pose_setpoint", ControlInterface::PoseSetpoint},
                                               {"waypoint", ControlInterface::Waypoint}}};
const EnumTable<Transport> kTransports{{{"ros2", Transport::Ros2},
                                        {"mavlink", Transport::Mavlink},
                                        {"serial", Transport::Serial},
                                        {"custom", Transport::Custom}}};
const EnumTable<LinkLostAction> kLinkLost{{{"stop", LinkLostAction::Stop},
                                           {"hold", LinkLostAction::Hold},
                                           {"finish_leg_then_stop", LinkLostAction::FinishLegThenStop},
                                           {"return_home", LinkLostAction::ReturnHome},
                                           {"land", LinkLostAction::Land}}};
const EnumTable<UncoveredMotion> kUncovered{{{"forbid", UncoveredMotion::Forbid},
                                             {"creep", UncoveredMotion::Creep}}};
const EnumTable<SensorKind> kSensorKinds{{{"range_2d", SensorKind::Range2d},
                                          {"range_3d", SensorKind::Range3d},
                                          {"depth", SensorKind::Depth},
                                          {"rgb", SensorKind::Rgb},
                                          {"imu", SensorKind::Imu},
                                          {"wheel_odom", SensorKind::WheelOdom},
                                          {"odometry", SensorKind::Odometry},
                                          {"contact", SensorKind::Contact}}};
const EnumTable<Accelerator> kAccelerators{{{"none", Accelerator::None},
                                            {"rk3588_npu", Accelerator::Rk3588Npu},
                                            {"hailo", Accelerator::Hailo},
                                            {"cuda", Accelerator::Cuda}}};
const EnumTable<Provenance> kProvenance{{{"declared", Provenance::Declared},
                                         {"cad", Provenance::Cad},
                                         {"calibrated", Provenance::Calibrated},
                                         {"llm_draft", Provenance::LlmDraft},
                                         {"llm_draft_reviewed", Provenance::LlmDraftReviewed}}};

const std::regex kIdPattern("^[a-z0-9][a-z0-9_-]{0,63}$");

// Reads a YAML document field by field and records every problem with its path.
class Reader {
 public:
  std::vector<std::string> problems;

  void fail(const std::string& path, const std::string& message) {
    problems.push_back(path + ": " + message);
  }

  bool expect_map(const YAML::Node& node, const std::string& path) {
    if (!node.IsMap()) {
      fail(path, "must be a mapping");
      return false;
    }
    return true;
  }

  void only_keys(const YAML::Node& node, const std::string& path, std::set<std::string> allowed) {
    if (!node.IsMap()) {
      return;
    }
    for (const auto& item : node) {
      const std::string key = item.first.as<std::string>();
      if (!allowed.count(key)) {
        fail(join(path, key), "unknown field");
      }
    }
  }

  YAML::Node child(const YAML::Node& node, const std::string& key, const std::string& path,
                   bool required) {
    if (!node.IsMap()) {
      return YAML::Node();
    }
    YAML::Node value = node[key];
    if (!value && required) {
      fail(join(path, key), "required");
    }
    return value;
  }

  std::optional<double> number(const YAML::Node& node, const std::string& key,
                               const std::string& path, bool required) {
    const YAML::Node value = child(node, key, path, required);
    if (!value) {
      return std::nullopt;
    }
    return as_number(value, join(path, key));
  }

  std::optional<double> as_number(const YAML::Node& value, const std::string& path) {
    if (!value.IsScalar()) {
      fail(path, "must be a number");
      return std::nullopt;
    }
    try {
      const double result = value.as<double>();
      if (!std::isfinite(result)) {
        fail(path, "must be finite");
        return std::nullopt;
      }
      return result;
    } catch (const YAML::Exception&) {
      fail(path, "must be a number");
      return std::nullopt;
    }
  }

  std::optional<bool> boolean(const YAML::Node& node, const std::string& key,
                              const std::string& path, bool required) {
    const YAML::Node value = child(node, key, path, required);
    if (!value) {
      return std::nullopt;
    }
    try {
      if (value.IsScalar() && (value.Scalar() == "true" || value.Scalar() == "false")) {
        return value.as<bool>();
      }
    } catch (const YAML::Exception&) {
    }
    fail(join(path, key), "must be true or false");
    return std::nullopt;
  }

  std::optional<std::string> text(const YAML::Node& node, const std::string& key,
                                  const std::string& path, bool required) {
    const YAML::Node value = child(node, key, path, required);
    if (!value) {
      return std::nullopt;
    }
    if (!value.IsScalar() || value.Scalar().empty()) {
      fail(join(path, key), "must be a non-empty string");
      return std::nullopt;
    }
    return value.Scalar();
  }

  template <typename Enum>
  std::optional<Enum> choice(const YAML::Node& node, const std::string& key,
                             const std::string& path, const EnumTable<Enum>& table,
                             bool required) {
    const auto value = text(node, key, path, required);
    if (!value) {
      return std::nullopt;
    }
    const auto parsed = table.parse(*value);
    if (!parsed) {
      fail(join(path, key), "must be one of: " + table.choices());
    }
    return parsed;
  }

  std::optional<std::vector<double>> vector(const YAML::Node& node, const std::string& key,
                                            const std::string& path, size_t size, bool required) {
    const YAML::Node value = child(node, key, path, required);
    if (!value) {
      return std::nullopt;
    }
    const std::string where = join(path, key);
    if (!value.IsSequence() || value.size() != size) {
      fail(where, "must be a list of " + std::to_string(size) + " numbers");
      return std::nullopt;
    }
    std::vector<double> out;
    for (size_t i = 0; i < size; ++i) {
      const auto n = as_number(value[i], where + "[" + std::to_string(i) + "]");
      if (!n) {
        return std::nullopt;
      }
      out.push_back(*n);
    }
    return out;
  }

  void positive(std::optional<double> value, const std::string& path) {
    if (value && *value <= 0.0) {
      fail(path, "must be greater than zero");
    }
  }

  void non_negative(std::optional<double> value, const std::string& path) {
    if (value && *value < 0.0) {
      fail(path, "must not be negative");
    }
  }

  static std::string join(const std::string& path, const std::string& key) {
    return path.empty() ? key : path + "." + key;
  }
};

Pose read_pose(Reader& r, const YAML::Node& node, const std::string& path) {
  Pose pose;
  if (!node) {
    return pose;
  }
  if (!r.expect_map(node, path)) {
    return pose;
  }
  r.only_keys(node, path, {"xyz_m", "rpy_rad"});
  if (auto xyz = r.vector(node, "xyz_m", path, 3, false)) {
    pose.xyz = Vec3((*xyz)[0], (*xyz)[1], (*xyz)[2]);
  }
  if (auto rpy = r.vector(node, "rpy_rad", path, 3, false)) {
    pose.rpy = Vec3((*rpy)[0], (*rpy)[1], (*rpy)[2]);
  }
  return pose;
}

void read_geometry(Reader& r, const YAML::Node& node, const std::string& path, GeometrySpec& out) {
  if (!r.expect_map(node, path)) {
    return;
  }
  r.only_keys(node, path, {"ground_z_m", "shapes", "frames"});
  out.ground_z_m = r.number(node, "ground_z_m", path, false);

  const YAML::Node shapes = r.child(node, "shapes", path, true);
  if (shapes) {
    if (!shapes.IsSequence() || shapes.size() == 0) {
      r.fail(path + ".shapes", "must be a non-empty list");
    } else {
      for (size_t i = 0; i < shapes.size(); ++i) {
        const std::string where = path + ".shapes[" + std::to_string(i) + "]";
        const YAML::Node s = shapes[i];
        if (!r.expect_map(s, where)) {
          continue;
        }
        Shape shape;
        const auto type = r.text(s, "type", where, true);
        shape.pose = read_pose(r, s["pose"], where + ".pose");
        if (type == std::optional<std::string>("box")) {
          r.only_keys(s, where, {"type", "pose", "size_m"});
          shape.type = ShapeType::Box;
          if (auto size = r.vector(s, "size_m", where, 3, true)) {
            shape.size = Vec3((*size)[0], (*size)[1], (*size)[2]);
            if (shape.size.minCoeff() <= 0.0) {
              r.fail(where + ".size_m", "every side must be greater than zero");
            }
          }
        } else if (type == std::optional<std::string>("cylinder")) {
          r.only_keys(s, where, {"type", "pose", "radius_m", "length_m"});
          shape.type = ShapeType::Cylinder;
          const auto radius = r.number(s, "radius_m", where, true);
          const auto length = r.number(s, "length_m", where, true);
          r.positive(radius, where + ".radius_m");
          r.positive(length, where + ".length_m");
          shape.radius = radius.value_or(0.0);
          shape.length = length.value_or(0.0);
        } else if (type == std::optional<std::string>("sphere")) {
          r.only_keys(s, where, {"type", "pose", "radius_m"});
          shape.type = ShapeType::Sphere;
          const auto radius = r.number(s, "radius_m", where, true);
          r.positive(radius, where + ".radius_m");
          shape.radius = radius.value_or(0.0);
        } else if (type) {
          r.fail(where + ".type", "must be one of: box, cylinder, sphere");
        }
        out.shapes.push_back(shape);
      }
    }
  }

  const YAML::Node frames = r.child(node, "frames", path, false);
  if (frames) {
    if (!r.expect_map(frames, path + ".frames")) {
      return;
    }
    for (const auto& item : frames) {
      const std::string name = item.first.as<std::string>();
      const std::string where = path + ".frames." + name;
      if (!std::regex_match(name, kIdPattern)) {
        r.fail(where, "frame names use lowercase letters, digits, '_' and '-'");
      }
      if (!item.second.IsMap()) {
        r.fail(where, "must be a mapping");
        continue;
      }
      out.frames[name] = read_pose(r, item.second, where);
    }
  }
}

void read_axis(Reader& r, const YAML::Node& limits, const std::string& axis,
               const std::string& path, AxisLimits& out) {
  const std::string where = path + "." + axis;
  const YAML::Node node = r.child(limits, axis, path, true);
  if (!node || !r.expect_map(node, where)) {
    return;
  }
  r.only_keys(node, where, {"velocity_mps", "accel_mps2", "decel_mps2"});
  if (auto v = r.vector(node, "velocity_mps", where, 2, true)) {
    out.velocity_mps = {(*v)[0], (*v)[1]};
    if (out.velocity_mps.min > out.velocity_mps.max) {
      r.fail(where + ".velocity_mps", "minimum is larger than maximum");
    } else if (out.velocity_mps.min > 0.0 || out.velocity_mps.max < 0.0) {
      r.fail(where + ".velocity_mps", "range must include zero so the body can stop");
    }
  }
  const bool moves = out.moves();
  const auto accel = r.number(node, "accel_mps2", where, moves);
  const auto decel = r.number(node, "decel_mps2", where, moves);
  r.positive(accel, where + ".accel_mps2");
  r.positive(decel, where + ".decel_mps2");
  out.accel_mps2 = accel.value_or(0.0);
  out.decel_mps2 = decel.value_or(0.0);
}

void read_motion(Reader& r, const YAML::Node& node, const std::string& path, MotionSpec& out) {
  if (!r.expect_map(node, path)) {
    return;
  }
  r.only_keys(node, path,
              {"space", "can_reverse", "can_strafe", "min_turn_radius_m", "collision_padding_m",
               "limits"});
  out.space = r.choice(node, "space", path, kMotionSpaces, true).value_or(MotionSpace::Planar);
  out.can_reverse = r.boolean(node, "can_reverse", path, true).value_or(false);
  out.can_strafe = r.boolean(node, "can_strafe", path, true).value_or(false);
  const auto radius = r.number(node, "min_turn_radius_m", path, false);
  r.non_negative(radius, path + ".min_turn_radius_m");
  out.min_turn_radius_m = radius.value_or(0.0);
  const auto padding = r.number(node, "collision_padding_m", path, true);
  r.non_negative(padding, path + ".collision_padding_m");
  out.collision_padding_m = padding.value_or(0.0);

  const std::string lpath = path + ".limits";
  const YAML::Node limits = r.child(node, "limits", path, true);
  if (!limits || !r.expect_map(limits, lpath)) {
    return;
  }
  r.only_keys(limits, lpath, {"x", "y", "z", "yaw"});
  read_axis(r, limits, "x", lpath, out.x);
  read_axis(r, limits, "y", lpath, out.y);
  read_axis(r, limits, "z", lpath, out.z);
  const YAML::Node yaw = r.child(limits, "yaw", lpath, true);
  if (yaw && r.expect_map(yaw, lpath + ".yaw")) {
    r.only_keys(yaw, lpath + ".yaw", {"rate_radps", "accel_radps2"});
    const auto rate = r.number(yaw, "rate_radps", lpath + ".yaw", true);
    const auto accel = r.number(yaw, "accel_radps2", lpath + ".yaw", true);
    r.positive(rate, lpath + ".yaw.rate_radps");
    r.positive(accel, lpath + ".yaw.accel_radps2");
    out.yaw_rate_radps = rate.value_or(0.0);
    out.yaw_accel_radps2 = accel.value_or(0.0);
  }
}

void read_control(Reader& r, const YAML::Node& node, const std::string& path, ControlSpec& out) {
  if (!r.expect_map(node, path)) {
    return;
  }
  r.only_keys(node, path,
              {"interface", "transport", "endpoint", "rate_hz", "latency_s",
               "controller_timeout_s"});
  out.interface = r.choice(node, "interface", path, kInterfaces, true)
                      .value_or(ControlInterface::Velocity);
  out.transport = r.choice(node, "transport", path, kTransports, true).value_or(Transport::Ros2);
  out.endpoint = r.text(node, "endpoint", path, true).value_or("");
  const auto rate = r.number(node, "rate_hz", path, true);
  const auto latency = r.number(node, "latency_s", path, true);
  const auto timeout = r.number(node, "controller_timeout_s", path, true);
  r.positive(rate, path + ".rate_hz");
  r.non_negative(latency, path + ".latency_s");
  r.positive(timeout, path + ".controller_timeout_s");
  out.rate_hz = rate.value_or(0.0);
  out.latency_s = latency.value_or(0.0);
  out.controller_timeout_s = timeout.value_or(0.0);
}

void read_failsafe(Reader& r, const YAML::Node& node, const std::string& path,
                   FailsafeSpec& out) {
  if (!r.expect_map(node, path)) {
    return;
  }
  r.only_keys(node, path,
              {"link_lost_after_s", "on_link_lost", "uncovered_motion", "creep_speed_mps"});
  const auto after = r.number(node, "link_lost_after_s", path, true);
  r.positive(after, path + ".link_lost_after_s");
  out.link_lost_after_s = after.value_or(0.0);
  out.on_link_lost =
      r.choice(node, "on_link_lost", path, kLinkLost, true).value_or(LinkLostAction::Stop);
  out.uncovered_motion = r.choice(node, "uncovered_motion", path, kUncovered, true)
                             .value_or(UncoveredMotion::Forbid);
  const bool creep = out.uncovered_motion == UncoveredMotion::Creep;
  const auto speed = r.number(node, "creep_speed_mps", path, creep);
  r.positive(speed, path + ".creep_speed_mps");
  out.creep_speed_mps = speed.value_or(0.0);
}

void read_sensors(Reader& r, const YAML::Node& node, const std::string& path,
                  std::vector<SensorSpec>& out) {
  if (!node.IsSequence()) {
    r.fail(path, "must be a list (it may be empty)");
    return;
  }
  for (size_t i = 0; i < node.size(); ++i) {
    const std::string where = path + "[" + std::to_string(i) + "]";
    const YAML::Node s = node[i];
    if (!r.expect_map(s, where)) {
      continue;
    }
    r.only_keys(s, where,
                {"id", "kind", "frame", "source", "rate_hz", "max_age_s", "range_m", "fov_deg"});
    SensorSpec sensor;
    sensor.id = r.text(s, "id", where, true).value_or("");
    if (!sensor.id.empty() && !std::regex_match(sensor.id, kIdPattern)) {
      r.fail(where + ".id", "ids use lowercase letters, digits, '_' and '-'");
    }
    sensor.kind = r.choice(s, "kind", where, kSensorKinds, true).value_or(SensorKind::Imu);
    sensor.frame = r.text(s, "frame", where, true).value_or("");
    const YAML::Node source = r.child(s, "source", where, true);
    if (source && r.expect_map(source, where + ".source")) {
      r.only_keys(source, where + ".source", {"transport", "endpoint"});
      sensor.transport =
          r.choice(source, "transport", where + ".source", kTransports, true).value_or(Transport::Ros2);
      sensor.endpoint = r.text(source, "endpoint", where + ".source", true).value_or("");
    }
    const auto rate = r.number(s, "rate_hz", where, true);
    r.positive(rate, where + ".rate_hz");
    sensor.rate_hz = rate.value_or(0.0);
    const auto age = r.number(s, "max_age_s", where, false);
    r.positive(age, where + ".max_age_s");
    sensor.max_age_s = age.value_or(sensor.rate_hz > 0.0 ? 3.0 / sensor.rate_hz : 0.0);

    const bool ranged = is_range_sensor(sensor.kind);
    const bool imaging = ranged || sensor.kind == SensorKind::Rgb;
    if (auto range = r.vector(s, "range_m", where, 2, ranged)) {
      sensor.range_m = Interval{(*range)[0], (*range)[1]};
      if ((*range)[0] < 0.0 || (*range)[0] >= (*range)[1]) {
        r.fail(where + ".range_m", "must be [min, max] with 0 <= min < max");
      }
    }
    const YAML::Node fov = r.child(s, "fov_deg", where, imaging);
    if (fov && r.expect_map(fov, where + ".fov_deg")) {
      r.only_keys(fov, where + ".fov_deg", {"h", "v"});
      sensor.fov_h_deg = r.number(fov, "h", where + ".fov_deg", true);
      sensor.fov_v_deg = r.number(fov, "v", where + ".fov_deg", true);
      if (sensor.fov_h_deg && (*sensor.fov_h_deg <= 0.0 || *sensor.fov_h_deg > 360.0)) {
        r.fail(where + ".fov_deg.h", "must be in (0, 360]");
      }
      if (sensor.fov_v_deg && (*sensor.fov_v_deg < 0.0 || *sensor.fov_v_deg > 180.0)) {
        r.fail(where + ".fov_deg.v", "must be in [0, 180]");
      }
    }
    out.push_back(sensor);
  }
}

// Resolves a provenance path such as "motion.limits.x.decel_mps2" or "sensors.front_lidar".
bool provenance_path_exists(const YAML::Node& root, const std::string& path) {
  std::vector<std::string> parts;
  std::stringstream stream(path);
  std::string part;
  while (std::getline(stream, part, '.')) {
    parts.push_back(part);
  }
  // yaml-cpp's operator= writes into the tree, so walk with reset() instead.
  YAML::Node node;
  node.reset(root);
  for (const std::string& key : parts) {
    const YAML::Node& current = node;
    YAML::Node next;
    bool found = false;
    if (current.IsSequence()) {
      for (const auto& item : current) {
        if (item.IsMap() && item["id"] && item["id"].Scalar() == key) {
          next.reset(item);
          found = true;
        }
      }
    } else if (current.IsMap() && current[key]) {
      next.reset(current[key]);
      found = true;
    }
    if (!found) {
      return false;
    }
    node.reset(next);
  }
  return !parts.empty();
}

void check_consistency(Reader& r, const Profile& p) {
  const MotionSpec& m = p.motion;
  if (m.space == MotionSpace::Planar) {
    if (!p.geometry.ground_z_m) {
      r.fail("geometry.ground_z_m", "required for planar bodies");
    }
    if (m.z.moves()) {
      r.fail("motion.limits.z.velocity_mps", "planar bodies cannot move vertically");
    }
    if (p.failsafe.on_link_lost == LinkLostAction::Land) {
      r.fail("failsafe.on_link_lost", "land is only valid for aerial bodies");
    }
  }
  if (!m.can_strafe && m.y.moves()) {
    r.fail("motion.limits.y.velocity_mps", "must be [0, 0] when can_strafe is false");
  }
  if (!m.can_reverse && m.x.velocity_mps.min < 0.0) {
    r.fail("motion.limits.x.velocity_mps", "minimum must be 0 when can_reverse is false");
  }
  if (!m.x.moves() && !m.y.moves() && !m.z.moves()) {
    r.fail("motion.limits", "at least one axis must allow motion");
  }
  if (p.failsafe.uncovered_motion == UncoveredMotion::Creep &&
      p.failsafe.creep_speed_mps > std::max(m.x.velocity_mps.max, m.y.velocity_mps.max)) {
    r.fail("failsafe.creep_speed_mps", "is faster than the body's top speed");
  }
  std::set<std::string> ids;
  for (size_t i = 0; i < p.sensors.size(); ++i) {
    const SensorSpec& s = p.sensors[i];
    const std::string where = "sensors[" + std::to_string(i) + "]";
    if (!s.id.empty() && !ids.insert(s.id).second) {
      r.fail(where + ".id", "duplicate sensor id '" + s.id + "'");
    }
    if (!s.frame.empty() && !p.geometry.frames.count(s.frame)) {
      r.fail(where + ".frame", "frame '" + s.frame + "' is not defined in geometry.frames");
    }
  }
}

Profile parse(const YAML::Node& root) {
  Reader r;
  Profile p;
  if (!root.IsMap()) {
    throw ProfileError({"profile: must be a mapping"});
  }
  r.only_keys(root, "",
              {"argus_profile", "id", "name", "description_file", "geometry", "motion", "control",
               "failsafe", "sensors", "compute", "provenance"});
  if (auto version = r.number(root, "argus_profile", "", true)) {
    if (*version != kProfileFormatVersion) {
      r.fail("argus_profile", "unsupported format version, expected " +
                                  std::to_string(kProfileFormatVersion));
    }
  }
  p.id = r.text(root, "id", "", true).value_or("");
  if (!p.id.empty() && !std::regex_match(p.id, kIdPattern)) {
    r.fail("id", "ids use lowercase letters, digits, '_' and '-'");
  }
  p.name = r.text(root, "name", "", false).value_or("");
  p.description_file = r.text(root, "description_file", "", false).value_or("");

  if (auto n = r.child(root, "geometry", "", true)) read_geometry(r, n, "geometry", p.geometry);
  if (auto n = r.child(root, "motion", "", true)) read_motion(r, n, "motion", p.motion);
  if (auto n = r.child(root, "control", "", true)) read_control(r, n, "control", p.control);
  if (auto n = r.child(root, "failsafe", "", true)) read_failsafe(r, n, "failsafe", p.failsafe);
  if (auto n = r.child(root, "sensors", "", true)) read_sensors(r, n, "sensors", p.sensors);

  if (auto n = r.child(root, "compute", "", true); n && r.expect_map(n, "compute")) {
    r.only_keys(n, "compute", {"board", "accelerator"});
    p.compute.board = r.text(n, "board", "compute", true).value_or("");
    p.compute.accelerator =
        r.choice(n, "accelerator", "compute", kAccelerators, true).value_or(Accelerator::None);
  }

  if (auto n = r.child(root, "provenance", "", false); n && r.expect_map(n, "provenance")) {
    for (const auto& item : n) {
      const std::string key = item.first.as<std::string>();
      const std::string where = "provenance." + key;
      if (!item.second.IsScalar()) {
        r.fail(where, "must be one of: " + kProvenance.choices());
        continue;
      }
      const auto value = kProvenance.parse(item.second.Scalar());
      if (!value) {
        r.fail(where, "must be one of: " + kProvenance.choices());
        continue;
      }
      if (key.rfind("provenance", 0) == 0 || !provenance_path_exists(root, key)) {
        r.fail(where, "does not name a field in this profile");
        continue;
      }
      p.provenance[key] = *value;
    }
  }

  if (r.problems.empty()) {
    check_consistency(r, p);
  }
  if (!r.problems.empty()) {
    throw ProfileError(r.problems);
  }
  return p;
}

YAML::Node load_yaml(const std::string& text) {
  try {
    return YAML::Load(text);
  } catch (const YAML::Exception& e) {
    throw ProfileError({std::string("not valid YAML: ") + e.what()});
  }
}

}  // namespace

ProfileError::ProfileError(std::vector<std::string> problems)
    : std::runtime_error(join_problems(problems)), problems_(std::move(problems)) {}

const SensorSpec* Profile::sensor(const std::string& sensor_id) const {
  for (const auto& s : sensors) {
    if (s.id == sensor_id) {
      return &s;
    }
  }
  return nullptr;
}

Profile load_profile_text(const std::string& yaml_text) { return parse(load_yaml(yaml_text)); }

Profile load_profile_file(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    throw ProfileError({"cannot read " + path});
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  return load_profile_text(buffer.str());
}

std::vector<std::string> validate_profile_text(const std::string& yaml_text) {
  try {
    load_profile_text(yaml_text);
    return {};
  } catch (const ProfileError& e) {
    return e.problems();
  }
}

bool is_range_sensor(SensorKind kind) {
  return kind == SensorKind::Range2d || kind == SensorKind::Range3d || kind == SensorKind::Depth;
}

std::string to_string(MotionSpace value) { return kMotionSpaces.name(value); }
std::string to_string(ControlInterface value) { return kInterfaces.name(value); }
std::string to_string(Transport value) { return kTransports.name(value); }
std::string to_string(LinkLostAction value) { return kLinkLost.name(value); }
std::string to_string(UncoveredMotion value) { return kUncovered.name(value); }
std::string to_string(SensorKind value) { return kSensorKinds.name(value); }
std::string to_string(Accelerator value) { return kAccelerators.name(value); }
std::string to_string(Provenance value) { return kProvenance.name(value); }

}  // namespace argus::capabilities

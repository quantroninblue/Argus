#include "argus/capabilities/capabilities.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace argus::capabilities {

namespace {

constexpr double kDeg = M_PI / 180.0;
constexpr int kBoundSegments = 64;

AxisCapability to_axis(const AxisLimits& limits) {
  return {limits.velocity_mps.min, limits.velocity_mps.max, limits.accel_mps2, limits.decel_mps2};
}

double shape_bounding_radius(const Shape& shape) {
  if (shape.type == ShapeType::Sphere) {
    return shape.pose.xyz.norm() + shape.radius;
  }
  double r = 0.0;
  for (const Vec3& p : shape.surface_points(kBoundSegments)) {
    r = std::max(r, p.norm());
  }
  if (shape.type == ShapeType::Cylinder) {
    // Rim points are polygon vertices; cover the arc between them.
    r += shape.radius * (1.0 - std::cos(M_PI / kBoundSegments));
  }
  return r;
}

CollisionCapability derive_collision(const Profile& profile) {
  CollisionCapability c;
  c.space = profile.motion.space;
  c.padding_m = profile.motion.collision_padding_m;
  c.aabb_min = Vec3::Constant(std::numeric_limits<double>::infinity());
  c.aabb_max = Vec3::Constant(-std::numeric_limits<double>::infinity());
  std::vector<Vec2> projected;
  for (const Shape& shape : profile.geometry.shapes) {
    for (const Vec3& p : shape.surface_points(kBoundSegments)) {
      c.aabb_min = c.aabb_min.cwiseMin(p);
      c.aabb_max = c.aabb_max.cwiseMax(p);
      projected.push_back(p.head<2>());
    }
    c.bounding_radius_m = std::max(c.bounding_radius_m, shape_bounding_radius(shape));
  }
  c.body_radius_m = c.bounding_radius_m;
  c.bounding_radius_m += c.padding_m;

  std::vector<Vec2> hull = convex_hull(projected);
  c.body_footprint = hull;
  if (c.padding_m > 0.0) {
    std::vector<Vec2> padded;
    for (const Vec2& v : hull) {
      for (int i = 0; i < 16; ++i) {
        const double a = 2.0 * M_PI * i / 16;
        // Scale so the polygon around each vertex encloses the padding circle.
        const double r = c.padding_m / std::cos(M_PI / 16);
        padded.emplace_back(v.x() + r * std::cos(a), v.y() + r * std::sin(a));
      }
    }
    hull = convex_hull(padded);
  }
  c.footprint = hull;
  c.inscribed_radius_m = c.bounding_radius_m;
  if (c.space == MotionSpace::Planar) {
    const size_t n = hull.size();
    for (size_t i = 0; i < n; ++i) {
      const Vec2 a = hull[i];
      const Vec2 e = hull[(i + 1) % n] - a;
      const double t = std::clamp(-a.dot(e) / std::max(e.squaredNorm(), 1e-12), 0.0, 1.0);
      c.inscribed_radius_m = std::min(c.inscribed_radius_m, (a + t * e).norm());
    }
  }
  return c;
}

struct RangeSensor {
  const SensorSpec* spec = nullptr;
  Eigen::Isometry3d to_sensor;
  Vec3 origin;
  Vec3 forward;
  double half_h = 0.0;
  double half_v = 0.0;
  bool full_circle = false;
  std::vector<const Shape*> occluders;

  bool sees(const Vec3& point) const {
    const Vec3 q = to_sensor * point;
    const double range = q.norm();
    if (range < spec->range_m->min || range > spec->range_m->max) {
      return false;
    }
    if (!full_circle && std::abs(std::atan2(q.y(), q.x())) > half_h + 1e-9) {
      return false;
    }
    // A scan plane is a degree or so thick; near the sensor that is thinner than the 2 cm the
    // heights are sampled at, so it would miss every sample. Give it at least half a step.
    if (std::abs(std::atan2(q.z(), q.head<2>().norm())) > half_v + 1e-9 && std::abs(q.z()) > 0.01) {
      return false;
    }
    const Vec3 ray = point - origin;
    for (const Shape* shape : occluders) {
      if (shape->intersect(origin, ray, 1e-6, 1.0 - 1e-6)) {
        return false;
      }
    }
    return true;
  }
};

std::vector<RangeSensor> range_sensors(const Profile& profile) {
  std::vector<RangeSensor> out;
  for (const SensorSpec& spec : profile.sensors) {
    if (!is_range_sensor(spec.kind) || !spec.range_m || !spec.fov_h_deg || !spec.fov_v_deg) {
      continue;
    }
    const Eigen::Isometry3d mount = profile.geometry.frames.at(spec.frame).transform();
    RangeSensor s;
    s.spec = &spec;
    s.to_sensor = mount.inverse();
    s.origin = mount.translation();
    s.forward = mount.linear() * Vec3::UnitX();
    s.half_h = 0.5 * *spec.fov_h_deg * kDeg;
    s.half_v = 0.5 * *spec.fov_v_deg * kDeg;
    s.full_circle = *spec.fov_h_deg >= 360.0;
    // A sensor sits inside its own housing; that housing does not block its view.
    for (const Shape& shape : profile.geometry.shapes) {
      if (!shape.contains(s.origin)) {
        s.occluders.push_back(&shape);
      }
    }
    out.push_back(s);
  }
  return out;
}

std::vector<HeightBand> height_bands(const Profile& profile, const CollisionCapability& collision,
                                     const DeriveOptions& options) {
  const double ground = profile.geometry.ground_z_m.value_or(0.0);
  const double low = options.min_obstacle_height_m;
  const double top = std::max(collision.aabb_max.z() - ground, low + options.band_height_m);
  std::vector<HeightBand> bands;
  for (double h = low; h < top - 1e-9;) {
    double high = h + options.band_height_m;
    if (top - high < 0.5 * options.band_height_m) {
      high = top;  // fold a thin remainder into the last band
    }
    bands.push_back({h, high});
    h = high;
  }
  return bands;
}

// Heights across one band, no more than 2 cm apart. A band counts as seen at a distance only
// when the sensors see all of them there: a single scan plane crossing the band does not see
// an obstacle below it.
std::vector<double> band_heights(const HeightBand& band, double ground) {
  const int n = std::max(2, static_cast<int>(std::ceil((band.high_m - band.low_m) / 0.02)) + 1);
  std::vector<double> z;
  for (int i = 0; i < n; ++i) {
    z.push_back(ground + band.low_m + (band.high_m - band.low_m) * i / (n - 1));
  }
  return z;
}

SensingCapability derive_planar_sensing(const Profile& profile,
                                        const CollisionCapability& collision,
                                        const std::vector<RangeSensor>& sensors,
                                        const DeriveOptions& options) {
  SensingCapability sensing;
  sensing.evaluation_range_m = options.evaluation_range_m;
  sensing.sample_step_m = options.sample_step_m;
  sensing.bands = height_bands(profile, collision, options);
  const double ground = profile.geometry.ground_z_m.value_or(0.0);
  const double low = options.min_obstacle_height_m;
  // Top of the column a declared obstacle height covers, capped at the top of the bands (the
  // body's own top, or one band height for a body lower than that).
  const double band_top = sensing.bands.empty() ? collision.aabb_max.z() - ground : sensing.bands.back().high_m;
  const double assumed_top = ground + std::min(profile.assumed_min_obstacle_height_m.value_or(0.0), band_top);
  const int samples = static_cast<int>(std::floor(options.evaluation_range_m / options.sample_step_m));

  for (double az = 0.0; az < 360.0 - 1e-9; az += options.azimuth_step_deg) {
    CoverageBin bin;
    bin.azimuth_deg = az;
    const Vec2 dir(std::cos(az * kDeg), std::sin(az * kDeg));
    bin.edge_distance_m = collision.edge_distance_m(az * kDeg);
    // Per band: where some height in it is seen (a scan plane crossing it), and where every
    // height in it is seen. Only the second makes a band count toward full-height coverage.
    const size_t nb = sensing.bands.size();
    std::vector<double> any_first(nb, kNever), any_last(nb, 0.0), all_first(nb, kNever), all_last(nb, 0.0);
    std::set<std::string> contributing;

    for (int i = 0; i <= samples; ++i) {
      const double s = i * options.sample_step_m;
      const Vec2 xy = (bin.edge_distance_m + s) * dir;
      auto seen_at = [&](double z) {
        for (const RangeSensor& sensor : sensors) {
          if (sensor.sees(Vec3(xy.x(), xy.y(), z))) {
            contributing.insert(sensor.spec->id);
            return true;
          }
        }
        return false;
      };
      // With a declared obstacle height, obstacles reach from the ground to at least that
      // height, so seeing any height up to it sees the whole column below it.
      bool column = false;
      if (assumed_top > ground + low) {
        for (double z : band_heights({low, assumed_top - ground}, ground)) {
          if (seen_at(z)) {
            column = true;
            break;
          }
        }
      }
      for (size_t b = 0; b < nb; ++b) {
        bool all = true;
        bool any = false;
        for (double z : band_heights(sensing.bands[b], ground)) {
          const bool seen = seen_at(z) || (column && z <= assumed_top + 1e-9);
          any = any || seen;
          all = all && seen;
        }
        if (any) {
          any_first[b] = std::min(any_first[b], s);
          any_last[b] = std::max(any_last[b], s);
        }
        if (all) {
          all_first[b] = std::min(all_first[b], s);
          all_last[b] = std::max(all_last[b], s);
        }
      }
    }

    bool every_band = true;
    double full_from = 0.0;
    double reach = kNever;
    for (size_t b = 0; b < nb; ++b) {
      if (any_first[b] != kNever) {
        bin.first_seen_m = std::min(bin.first_seen_m, any_first[b]);
        bin.max_seen_m = std::max(bin.max_seen_m, any_last[b]);
      }
      if (all_first[b] == kNever) {
        every_band = false;
        bin.unseen_bands.push_back(sensing.bands[b]);
        continue;
      }
      full_from = std::max(full_from, all_first[b]);
      reach = std::min(reach, all_last[b]);
    }
    if (every_band) {
      bin.full_height_from_m = full_from;
      bin.full_height_reach_m = full_from <= options.max_near_gap_m + 1e-9 ? reach : 0.0;
    }
    bin.sensors.assign(contributing.begin(), contributing.end());
    sensing.bins.push_back(bin);
  }
  return sensing;
}

SensingCapability derive_aerial_sensing(const CollisionCapability& collision,
                                        const std::vector<RangeSensor>& sensors,
                                        const DeriveOptions& options) {
  SensingCapability sensing;
  sensing.evaluation_range_m = options.evaluation_range_m;
  sensing.sample_step_m = options.sample_step_m;
  const int samples = static_cast<int>(std::floor(options.evaluation_range_m / options.sample_step_m));

  for (double el = -90.0; el <= 90.0 + 1e-9; el += options.elevation_step_deg) {
    for (double az = 0.0; az < 360.0 - 1e-9; az += options.azimuth_step_deg) {
      CoverageBin bin;
      bin.azimuth_deg = az;
      bin.elevation_deg = el;
      const Vec3 dir(std::cos(el * kDeg) * std::cos(az * kDeg),
                     std::cos(el * kDeg) * std::sin(az * kDeg), std::sin(el * kDeg));
      bin.edge_distance_m = collision.edge_distance_m(az * kDeg, el * kDeg);
      std::set<std::string> contributing;
      for (int i = 0; i <= samples; ++i) {
        const double s = i * options.sample_step_m;
        const Vec3 p = (bin.edge_distance_m + s) * dir;
        for (const RangeSensor& sensor : sensors) {
          if (sensor.sees(p)) {
            bin.first_seen_m = std::min(bin.first_seen_m, s);
            bin.max_seen_m = std::max(bin.max_seen_m, s);
            contributing.insert(sensor.spec->id);
          }
        }
      }
      bin.full_height_from_m = bin.first_seen_m;
      if (bin.seen() && bin.first_seen_m <= options.max_near_gap_m + 1e-9) {
        bin.full_height_reach_m = bin.max_seen_m;
      }
      bin.sensors.assign(contributing.begin(), contributing.end());
      sensing.bins.push_back(bin);
    }
  }
  return sensing;
}

LocalizationCapability derive_localization(const Profile& profile) {
  LocalizationCapability loc;
  for (const SensorSpec& s : profile.sensors) {
    switch (s.kind) {
      case SensorKind::Imu:
        loc.inertial = true;
        break;
      case SensorKind::WheelOdom:
        loc.wheel_odometry = true;
        break;
      case SensorKind::Odometry:
        loc.external_odometry = true;
        break;
      case SensorKind::Rgb:
        loc.visual = true;
        break;
      case SensorKind::Depth:
        loc.visual = true;
        loc.range = true;
        break;
      case SensorKind::Range2d:
      case SensorKind::Range3d:
        loc.range = true;
        break;
      case SensorKind::Contact:
        break;
    }
  }
  return loc;
}

std::vector<std::string> inference_backends(Accelerator accelerator) {
  switch (accelerator) {
    case Accelerator::Rk3588Npu:
      return {"rknn", "onnx_cpu"};
    case Accelerator::Hailo:
      return {"hailo", "onnx_cpu"};
    case Accelerator::Cuda:
      return {"tensorrt", "onnx_cpu"};
    case Accelerator::None:
      break;
  }
  return {"onnx_cpu"};
}

}  // namespace

const AxisCapability& MotionCapability::axis(Axis a) const {
  switch (a) {
    case Axis::X:
      return x;
    case Axis::Y:
      return y;
    case Axis::Z:
      return z;
  }
  return x;
}

double MotionCapability::stopping_distance_m(Axis a, double speed_mps) const {
  const double v = std::abs(speed_mps);
  if (v == 0.0) {
    return 0.0;
  }
  const AxisCapability& limits = axis(a);
  if (limits.decel_mps2 <= 0.0) {
    return kNever;
  }
  return v * latency_s + v * v / (2.0 * limits.decel_mps2);
}

double CollisionCapability::edge_distance_m(double azimuth_rad, double elevation_rad) const {
  if (space == MotionSpace::Aerial) {
    return bounding_radius_m;
  }
  (void)elevation_rad;
  return ray_exit_distance(footprint, Vec2(std::cos(azimuth_rad), std::sin(azimuth_rad)));
}

double SensingCapability::seen_fraction() const {
  if (bins.empty()) {
    return 0.0;
  }
  const auto seen = std::count_if(bins.begin(), bins.end(), [](const CoverageBin& b) { return b.seen(); });
  return static_cast<double>(seen) / bins.size();
}

double SensingCapability::full_height_fraction() const {
  if (bins.empty()) {
    return 0.0;
  }
  const auto full = std::count_if(bins.begin(), bins.end(), [](const CoverageBin& b) {
    return b.full_height_from_m != kNever;
  });
  return static_cast<double>(full) / bins.size();
}

std::vector<std::string> LocalizationCapability::sources() const {
  std::vector<std::string> out;
  if (inertial) out.push_back("inertial");
  if (wheel_odometry) out.push_back("wheel_odometry");
  if (external_odometry) out.push_back("external_odometry");
  if (visual) out.push_back("visual");
  if (range) out.push_back("range");
  return out;
}

Capabilities derive_capabilities(const Profile& profile, const DeriveOptions& options) {
  Capabilities caps;
  caps.min_obstacle_height_m = options.min_obstacle_height_m;
  caps.assumed_obstacle_height_m = profile.assumed_min_obstacle_height_m.value_or(0.0);
  caps.assumed_clear_at_start_m = profile.assumed_clear_at_start_m.value_or(0.0);
  caps.profile_id = profile.id;
  caps.control = profile.control;
  caps.failsafe = profile.failsafe;

  const MotionSpec& m = profile.motion;
  caps.motion.space = m.space;
  caps.motion.can_reverse = m.can_reverse;
  caps.motion.can_strafe = m.can_strafe;
  caps.motion.min_turn_radius_m = m.min_turn_radius_m;
  caps.motion.can_turn_in_place = m.min_turn_radius_m == 0.0 && m.yaw_rate_radps > 0.0;
  caps.motion.can_hover = m.space == MotionSpace::Aerial;
  caps.motion.x = to_axis(m.x);
  caps.motion.y = to_axis(m.y);
  caps.motion.z = to_axis(m.z);
  caps.motion.yaw_rate_max_radps = m.yaw_rate_radps;
  caps.motion.yaw_accel_radps2 = m.yaw_accel_radps2;
  caps.motion.latency_s = profile.control.latency_s;

  caps.collision = derive_collision(profile);
  const std::vector<RangeSensor> sensors = range_sensors(profile);
  caps.sensing = m.space == MotionSpace::Planar
                     ? derive_planar_sensing(profile, caps.collision, sensors, options)
                     : derive_aerial_sensing(caps.collision, sensors, options);
  caps.localization = derive_localization(profile);
  caps.compute.board = profile.compute.board;
  caps.compute.accelerator = profile.compute.accelerator;
  caps.compute.inference_backends = inference_backends(profile.compute.accelerator);

  for (const auto& [field, source] : profile.provenance) {
    if (source == Provenance::LlmDraft) {
      caps.unreviewed_fields.push_back(field);
    }
  }
  caps.safety_ready = caps.unreviewed_fields.empty();
  return caps;
}

}  // namespace argus::capabilities

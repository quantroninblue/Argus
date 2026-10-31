#pragma once

#include <cstdint>
#include <vector>

#include "argus/capabilities/profile.hpp"
#include "argus/node/ray_pattern.hpp"

// Conversions from what ROS 2 drivers publish to Argo's frames, on plain data so they can be
// tested without ROS. Every output is a range per ray of the sensor's RayPattern, row-major:
// metres for a return, +inf for no return within range, NaN where the sensor said nothing
// usable. A pattern ray covers a patch of the sensor's view; the nearest return in that patch
// is taken, so a thin obstacle between pattern rays is not lost.
namespace argus::argo_ros {

// Planar scan, angles in radians in the sensor frame (0 along x, counter-clockwise).
struct Scan {
  double angle_min = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
  std::vector<float> ranges;
};

std::vector<float> scan_to_pattern(const Scan& scan, const capabilities::SensorSpec& sensor,
                                   const node::RayPattern& pattern);

// Points in the sensor frame. Pattern rays with no point in their patch are NaN, not free:
// many lidar drivers leave out rays that returned nothing, but an absent point proves nothing.
std::vector<float> points_to_pattern(const std::vector<float>& xyz, const capabilities::SensorSpec& sensor,
                                     const node::RayPattern& pattern);

// Depth image (metres along the optical axis, row-major) with pinhole intrinsics. Zero and
// NaN are invalid, +inf is beyond range, as in REP 118.
struct DepthImage {
  int width = 0;
  int height = 0;
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  std::vector<float> depth;
};

std::vector<float> depth_to_pattern(const DepthImage& image, const capabilities::SensorSpec& sensor,
                                    const node::RayPattern& pattern);

// Yaw from a unit quaternion.
double yaw_of(double x, double y, double z, double w);

}  // namespace argus::argo_ros

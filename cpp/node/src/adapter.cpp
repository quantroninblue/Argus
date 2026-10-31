#include "argus/node/adapter.hpp"

#include <cmath>

namespace argus::node {

using capabilities::ControlInterface;

namespace {

double braking_offset(double v, double decel) {
  return decel > 0.0 ? v * std::abs(v) / (2.0 * decel) : 0.0;
}

}  // namespace

bool interface_supported(ControlInterface interface) {
  return interface == ControlInterface::Velocity || interface == ControlInterface::PoseSetpoint;
}

Command make_command(ControlInterface interface, const capabilities::MotionCapability& motion,
                     const Odometry& odom, const Twist& velocity, double now, double lead_s) {
  Command cmd;
  cmd.stamp = now;
  cmd.stop = velocity.is_zero();
  if (interface == ControlInterface::Velocity) {
    cmd.velocity = velocity;
    return cmd;
  }
  const Pose4& pose = odom.pose;
  if (cmd.stop) {
    const Twist& v = odom.twist;
    const Vec3 rest(braking_offset(v.linear.x(), motion.x.decel_mps2),
                    braking_offset(v.linear.y(), motion.y.decel_mps2),
                    braking_offset(v.linear.z(), motion.z.decel_mps2));
    cmd.setpoint.position = pose.position + pose.rotate(rest);
    cmd.setpoint.yaw = pose.yaw;
    return cmd;
  }
  cmd.setpoint_velocity = pose.rotate(velocity.linear);
  cmd.setpoint_yaw_rate = velocity.yaw_rate;
  cmd.setpoint.position = pose.position + cmd.setpoint_velocity * lead_s;
  cmd.setpoint.yaw = wrap_angle(pose.yaw + velocity.yaw_rate * lead_s);
  return cmd;
}

}  // namespace argus::node

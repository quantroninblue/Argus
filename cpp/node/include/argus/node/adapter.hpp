#pragma once

#include "argus/capabilities/capabilities.hpp"
#include "argus/node/types.hpp"

namespace argus::node {

bool interface_supported(capabilities::ControlInterface interface);

// Turns an approved velocity into a command for the body's control interface.
//
// Velocity: the base-frame velocity as is.
// Pose setpoint: a pose `lead_s` ahead along the velocity, with the velocity as feed-forward.
// A stop holds the point where the body will come to rest when braking from its measured
// velocity, so a flight controller brakes to a hover instead of flying back.
Command make_command(capabilities::ControlInterface interface, const capabilities::MotionCapability& motion,
                     const Odometry& odom, const Twist& velocity, double now, double lead_s);

}  // namespace argus::node

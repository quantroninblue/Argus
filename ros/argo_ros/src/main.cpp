// argo: Argo on a ROS 2 robot.
//
//   argo --config /etc/argo/argo.yaml [--setup]
//
// --setup runs Argo for setup's motion tests: no hub, and the robot moves only on velocities
// published to /argo/setup/cmd_vel (geometry_msgs/Twist, each good for three control periods),
// every one checked by the safety supervisor. Between requests Argo sends nothing, so the
// robot's own controller timeout is what stops it. The supervisor's flags go out on
// /argo/setup/flags (std_msgs/String) ten times a second.
//
// The config file is the one load_argo_config() reads, plus an optional ros section:
//
//   ros:
//     twist_stamped: true     # publish geometry_msgs/TwistStamped
//     base_frame: base_link
//
// SIGINT and SIGTERM stop the control loop, which then keeps sending stop commands briefly.

#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include <yaml-cpp/yaml.h>

#include "argus/argo/config.hpp"
#include "argus/argo_ros/ros_driver.hpp"

namespace {

argus::argo::Runtime* running = nullptr;

void on_signal(int) {
  if (running) {
    running->stop();
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string path;
  bool setup = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--config" && i + 1 < argc) {
      path = argv[i + 1];
    } else if (std::string(argv[i]) == "--setup") {
      setup = true;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr, "usage: argo --config <argo.yaml> [--setup]\n");
    return 2;
  }
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  int status = 0;
  try {
    argus::argo::ArgoFiles files = argus::argo::load_argo_config(path);
    files.runtime.setup_mode = setup;
    argus::argo_ros::RosDriverConfig ros;
    const YAML::Node extra = YAML::LoadFile(path)["ros"];
    if (extra) {
      ros.twist_stamped = extra["twist_stamped"] && extra["twist_stamped"].as<bool>();
      if (extra["base_frame"]) ros.base_frame = extra["base_frame"].as<std::string>();
    }
    auto node = std::make_shared<rclcpp::Node>("argo");
    argus::argo_ros::RosDriver driver(node, files.profile, ros);
    argus::argo::Runtime runtime(files.profile, files.runtime, driver, [&driver] { return driver.now(); });
    driver.bind(runtime.core());

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr setup_requests;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr setup_flags;
    rclcpp::TimerBase::SharedPtr setup_timer;
    if (setup) {
      const double hold = 3.0 * runtime.period_s();
      setup_requests = node->create_subscription<geometry_msgs::msg::Twist>(
          "/argo/setup/cmd_vel", 10, [&runtime, hold](geometry_msgs::msg::Twist::ConstSharedPtr m) {
            argus::node::Twist t;
            t.linear = argus::node::Vec3(m->linear.x, m->linear.y, m->linear.z);
            t.yaw_rate = m->angular.z;
            runtime.request_velocity(t, hold);
          });
      setup_flags = node->create_publisher<std_msgs::msg::String>("/argo/setup/flags", 10);
      setup_timer = node->create_wall_timer(std::chrono::milliseconds(100), [&runtime, &setup_flags] {
        std_msgs::msg::String msg;
        for (const std::string& name : argus::node::flag_names(runtime.flags())) {
          msg.data += (msg.data.empty() ? "" : " ") + name;
        }
        setup_flags->publish(msg);
      });
    }
    running = &runtime;
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    runtime.run();
    running = nullptr;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "argo: %s\n", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}

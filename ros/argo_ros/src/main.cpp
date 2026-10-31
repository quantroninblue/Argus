// argo: Argo on a ROS 2 robot.
//
//   argo --config /etc/argo/argo.yaml
//
// The config file is the one load_argo_config() reads, plus an optional ros section:
//
//   ros:
//     twist_stamped: true     # publish geometry_msgs/TwistStamped
//     base_frame: base_link
//
// SIGINT and SIGTERM stop the control loop, which then keeps sending stop commands briefly.

#include <csignal>
#include <cstdio>
#include <string>

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
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--config") {
      path = argv[i + 1];
    }
  }
  if (path.empty()) {
    std::fprintf(stderr, "usage: argo --config <argo.yaml>\n");
    return 2;
  }
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  int status = 0;
  try {
    const argus::argo::ArgoFiles files = argus::argo::load_argo_config(path);
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

#pragma once

#include <map>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "argus/argo/runtime.hpp"
#include "argus/argo_ros/convert.hpp"

namespace argus::argo_ros {

struct RosDriverConfig {
  // Publish the command as TwistStamped rather than Twist.
  bool twist_stamped = false;
  std::string base_frame = "base_link";
  // Camera intrinsics topic for a depth sensor: the image topic with its last name replaced.
  std::string camera_info_name = "camera_info";
};

// Argo's driver for robots that run ROS 2. It subscribes to every sensor and odometry topic in
// the profile, turns them into Argo frames on its own executor thread, and publishes the
// velocity command on the profile's control endpoint. Message stamps and now() use the node's
// clock, so Gazebo's simulated time works as well as the wall clock.
class RosDriver : public argo::Driver {
 public:
  RosDriver(rclcpp::Node::SharedPtr node, const capabilities::Profile& profile, RosDriverConfig config = {});
  ~RosDriver() override;

  // Reads each range sensor's ray pattern. Call before start().
  void bind(const node::Node& core);

  void start(argo::Inbox& inbox) override;
  void send(const node::Command& command) override;
  void stop() override;

  double now() const { return node_->now().seconds(); }
  // Topics subscribed so far, for start-up checks and logs.
  std::vector<std::string> topics() const;

 private:
  struct Range {
    const capabilities::SensorSpec* spec = nullptr;
    node::RayPattern pattern;
    rclcpp::SubscriptionBase::SharedPtr sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
    DepthImage intrinsics;
    bool have_intrinsics = false;
  };

  void push(const std::string& id, double stamp, std::vector<float> ranges);

  rclcpp::Node::SharedPtr node_;
  const capabilities::Profile& profile_;
  RosDriverConfig config_;
  std::map<std::string, Range> ranges_;
  const capabilities::SensorSpec* odom_spec_ = nullptr;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_pub_;
  argo::Inbox* inbox_ = nullptr;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_;
};

}  // namespace argus::argo_ros

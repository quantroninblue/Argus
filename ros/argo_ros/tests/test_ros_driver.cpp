#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <thread>

#include "argus/argo_ros/ros_driver.hpp"

namespace ar = argus::argo_ros;
namespace caps = argus::capabilities;
using namespace std::chrono_literals;

namespace {

class RosDriverTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    // Keep the test's topics on this machine and away from anything else running ROS 2.
    setenv("ROS_DOMAIN_ID", "87", 1);
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

template <class Done>
bool wait_for(Done done, std::chrono::milliseconds limit = 5000ms) {
  const auto until = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < until) {
    if (done()) return true;
    std::this_thread::sleep_for(20ms);
  }
  return false;
}

}  // namespace

TEST_F(RosDriverTest, SensorsAndOdometryReachArgoAndTheCommandGoesOut) {
  const caps::Profile profile =
      caps::load_profile_file(std::string(ARGUS_EXAMPLES_DIR) + "/planar_differential.yaml");
  argus::node::Node core(profile);
  ar::RosDriver driver(std::make_shared<rclcpp::Node>("argo_under_test"), profile);
  driver.bind(core);
  argus::argo::Inbox inbox;
  driver.start(inbox);

  // A stand-in robot on the profile's topics.
  auto robot = std::make_shared<rclcpp::Node>("fake_robot");
  auto scan_pub = robot->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
  auto odom_pub = robot->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS());
  geometry_msgs::msg::Twist last_cmd;
  int cmds = 0;
  auto cmd_sub = robot->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [&](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        last_cmd = *m;
        ++cmds;
      });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(robot);
  std::thread spin([&] { exec.spin(); });

  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp.sec = 12;
  scan.angle_min = -2.356f;
  scan.angle_increment = 0.0043633f;
  scan.range_min = 0.1f;
  scan.range_max = 12.0f;
  scan.ranges.assign(1081, 4.0f);
  nav_msgs::msg::Odometry odom;
  odom.header.stamp.sec = 12;
  odom.pose.pose.position.x = 1.5;
  odom.pose.pose.orientation.z = std::sin(0.25);
  odom.pose.pose.orientation.w = std::cos(0.25);
  odom.twist.twist.linear.x = 0.2;

  std::vector<argus::node::Odometry> got_odom;
  std::vector<argus::node::RangeFrame> got_frames;
  bool have_scan = false, have_odom = false;
  argus::node::RangeFrame frame;
  argus::node::Odometry o;
  ASSERT_TRUE(wait_for([&] {
    scan_pub->publish(scan);
    odom_pub->publish(odom);
    inbox.take(got_odom, got_frames);
    if (!got_frames.empty()) { frame = got_frames.back(); have_scan = true; }
    if (!got_odom.empty()) { o = got_odom.back(); have_odom = true; }
    return have_scan && have_odom;
  }));
  EXPECT_EQ(frame.sensor_id, "front_lidar");
  EXPECT_DOUBLE_EQ(frame.stamp, 12.0);
  ASSERT_EQ(frame.ranges.size(), core.ray_pattern("front_lidar").size());
  EXPECT_FLOAT_EQ(frame.ranges[frame.ranges.size() / 2], 4.0f);
  EXPECT_DOUBLE_EQ(o.pose.position.x(), 1.5);
  EXPECT_NEAR(o.pose.yaw, 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(o.twist.linear.x(), 0.2);

  argus::node::Command cmd;
  cmd.stop = false;
  cmd.velocity.linear = argus::node::Vec3(0.3, 0.0, 0.0);
  cmd.velocity.yaw_rate = -0.4;
  ASSERT_TRUE(wait_for([&] {
    driver.send(cmd);
    return cmds > 0 && last_cmd.linear.x > 0.0;
  }));
  EXPECT_DOUBLE_EQ(last_cmd.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(last_cmd.angular.z, -0.4);

  argus::node::Command stop;
  ASSERT_TRUE(wait_for([&] {
    driver.send(stop);
    return last_cmd.linear.x == 0.0 && last_cmd.angular.z == 0.0;
  }));

  exec.cancel();
  spin.join();
  driver.stop();
}

#include "argus/argo_ros/ros_driver.hpp"

#include <cstring>
#include <stdexcept>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace argus::argo_ros {

using capabilities::SensorKind;

namespace {

double seconds(const builtin_interfaces::msg::Time& t) { return t.sec + 1e-9 * t.nanosec; }

std::string sibling(const std::string& topic, const std::string& name) {
  const auto slash = topic.find_last_of('/');
  return slash == std::string::npos ? name : topic.substr(0, slash + 1) + name;
}

// Depth in metres from 32FC1 (metres) or 16UC1 (millimetres, 0 invalid).
bool depth_metres(const sensor_msgs::msg::Image& msg, std::vector<float>& out) {
  const size_t n = static_cast<size_t>(msg.width) * msg.height;
  out.resize(n);
  if (msg.encoding == "32FC1") {
    for (size_t r = 0; r < msg.height; ++r) {
      std::memcpy(&out[r * msg.width], &msg.data[r * msg.step], msg.width * sizeof(float));
    }
    return true;
  }
  if (msg.encoding == "16UC1") {
    for (size_t r = 0; r < msg.height; ++r) {
      for (size_t c = 0; c < msg.width; ++c) {
        uint16_t mm;
        std::memcpy(&mm, &msg.data[r * msg.step + c * 2], 2);
        out[r * msg.width + c] = mm == 0 ? 0.0f : 1e-3f * mm;
      }
    }
    return true;
  }
  return false;
}

}  // namespace

RosDriver::RosDriver(rclcpp::Node::SharedPtr node, const capabilities::Profile& profile, RosDriverConfig config)
    : node_(std::move(node)), profile_(profile), config_(std::move(config)) {
  if (profile_.control.interface != capabilities::ControlInterface::Velocity) {
    throw std::runtime_error("control.interface: only velocity is supported over ROS 2 so far");
  }
  for (const auto& s : profile_.sensors) {
    if (s.transport != capabilities::Transport::Ros2) {
      continue;
    }
    if (s.kind == SensorKind::Range2d || s.kind == SensorKind::Range3d || s.kind == SensorKind::Depth) {
      ranges_[s.id].spec = &s;
    }
    // Odometry from the robot's own estimator wins over raw wheel odometry.
    if (s.kind == SensorKind::Odometry || (s.kind == SensorKind::WheelOdom && !odom_spec_)) {
      odom_spec_ = &s;
    }
  }
  if (!odom_spec_) {
    throw std::runtime_error("sensors: no odometry or wheel_odom source on ROS 2");
  }
  if (config_.twist_stamped) {
    stamped_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(profile_.control.endpoint, 1);
  } else {
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(profile_.control.endpoint, 1);
  }
}

RosDriver::~RosDriver() { stop(); }

void RosDriver::bind(const node::Node& core) {
  for (auto& [id, r] : ranges_) {
    r.pattern = core.ray_pattern(id);
  }
}

void RosDriver::push(const std::string& id, double stamp, std::vector<float> ranges) {
  if (inbox_) {
    inbox_->push(node::RangeFrame{stamp, id, std::move(ranges)});
  }
}

void RosDriver::start(argo::Inbox& inbox) {
  inbox_ = &inbox;
  const auto qos = rclcpp::SensorDataQoS();
  for (auto& [id, r] : ranges_) {
    const std::string sid = id;
    Range* range = &r;
    switch (r.spec->kind) {
      case SensorKind::Range2d:
        r.sub = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            r.spec->endpoint, qos, [this, sid, range](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
              Scan scan{m->angle_min, m->angle_increment, m->range_min, m->range_max, m->ranges};
              push(sid, seconds(m->header.stamp), scan_to_pattern(scan, *range->spec, range->pattern));
            });
        break;
      case SensorKind::Range3d:
        r.sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            r.spec->endpoint, qos, [this, sid, range](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
              std::vector<float> xyz;
              xyz.reserve(static_cast<size_t>(m->width) * m->height * 3);
              sensor_msgs::PointCloud2ConstIterator<float> x(*m, "x"), y(*m, "y"), z(*m, "z");
              for (; x != x.end(); ++x, ++y, ++z) {
                xyz.push_back(*x);
                xyz.push_back(*y);
                xyz.push_back(*z);
              }
              push(sid, seconds(m->header.stamp), points_to_pattern(xyz, *range->spec, range->pattern));
            });
        break;
      case SensorKind::Depth:
        r.info_sub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
            sibling(r.spec->endpoint, config_.camera_info_name), qos,
            [range](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
              range->intrinsics.fx = m->k[0];
              range->intrinsics.fy = m->k[4];
              range->intrinsics.cx = m->k[2];
              range->intrinsics.cy = m->k[5];
              range->have_intrinsics = m->k[0] > 0.0 && m->k[4] > 0.0;
            });
        r.sub = node_->create_subscription<sensor_msgs::msg::Image>(
            r.spec->endpoint, qos, [this, sid, range](sensor_msgs::msg::Image::ConstSharedPtr m) {
              if (!range->have_intrinsics) {
                return;  // nothing can be placed without the camera model
              }
              DepthImage img = range->intrinsics;
              img.width = static_cast<int>(m->width);
              img.height = static_cast<int>(m->height);
              if (!depth_metres(*m, img.depth)) {
                RCLCPP_WARN_ONCE(node_->get_logger(), "%s: unsupported depth encoding %s", sid.c_str(),
                                 m->encoding.c_str());
                return;
              }
              push(sid, seconds(m->header.stamp), depth_to_pattern(img, *range->spec, range->pattern));
            });
        break;
      default:
        break;
    }
  }
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_spec_->endpoint, qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        node::Odometry o;
        o.stamp = seconds(m->header.stamp);
        const auto& p = m->pose.pose.position;
        const auto& q = m->pose.pose.orientation;
        o.pose.position = node::Vec3(p.x, p.y, p.z);
        o.pose.yaw = yaw_of(q.x, q.y, q.z, q.w);
        // nav_msgs/Odometry carries the twist in the child (base) frame, as Argo does.
        const auto& t = m->twist.twist;
        o.twist.linear = node::Vec3(t.linear.x, t.linear.y, t.linear.z);
        o.twist.yaw_rate = t.angular.z;
        inbox_->push(o);
      });
  executor_.add_node(node_);
  spin_ = std::thread([this] { executor_.spin(); });
}

void RosDriver::send(const node::Command& command) {
  geometry_msgs::msg::Twist t;
  if (!command.stop) {
    t.linear.x = command.velocity.linear.x();
    t.linear.y = command.velocity.linear.y();
    t.linear.z = command.velocity.linear.z();
    t.angular.z = command.velocity.yaw_rate;
  }
  if (stamped_pub_) {
    geometry_msgs::msg::TwistStamped s;
    s.header.stamp = node_->now();
    s.header.frame_id = config_.base_frame;
    s.twist = t;
    stamped_pub_->publish(s);
  } else {
    twist_pub_->publish(t);
  }
}

void RosDriver::stop() {
  if (spin_.joinable()) {
    executor_.cancel();
    spin_.join();
    executor_.remove_node(node_);
  }
}

std::vector<std::string> RosDriver::topics() const {
  std::vector<std::string> out;
  for (const auto& [id, r] : ranges_) {
    if (r.sub) out.push_back(r.sub->get_topic_name());
  }
  if (odom_sub_) out.push_back(odom_sub_->get_topic_name());
  return out;
}

}  // namespace argus::argo_ros

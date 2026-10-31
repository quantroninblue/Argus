#include <gtest/gtest.h>

#include "argus/capabilities/capabilities.hpp"
#include "argus/node/motion.hpp"

using namespace argus::node;
namespace caps = argus::capabilities;

namespace {

caps::MotionCapability differential() {
  caps::MotionCapability m;
  m.x = {-0.3, 1.0, 0.8, 1.2};
  m.yaw_rate_max_radps = 1.5;
  m.yaw_accel_radps2 = 3.0;
  m.latency_s = 0.08;
  return m;
}

}  // namespace

TEST(Motion, ClampRespectsWhatTheBodyCannotDo) {
  caps::MotionCapability m = differential();
  m.min_turn_radius_m = 1.0;
  const MotionModel model(m);
  Twist t;
  t.linear = Vec3(2.0, 0.5, 0.3);
  t.yaw_rate = 5.0;
  const Twist c = model.clamp(t);
  EXPECT_DOUBLE_EQ(c.linear.x(), 1.0);
  EXPECT_DOUBLE_EQ(c.linear.y(), 0.0);
  EXPECT_DOUBLE_EQ(c.linear.z(), 0.0);
  EXPECT_DOUBLE_EQ(c.yaw_rate, 1.0);  // |vx| / turn radius
}

TEST(Motion, AccelerateRespectsLimits) {
  const MotionModel model(differential());
  Twist target;
  target.linear.x() = 1.0;
  const Twist v = model.accelerate(Twist{}, target, 0.1);
  EXPECT_NEAR(v.linear.x(), 0.08, 1e-12);
  Twist moving;
  moving.linear.x() = 1.0;
  EXPECT_NEAR(model.accelerate(moving, Twist{}, 0.1).linear.x(), 0.88, 1e-12);
}

TEST(Motion, StoppingPathMatchesStoppingDistance) {
  const caps::MotionCapability m = differential();
  const MotionModel model(m);
  Twist v;
  v.linear.x() = 1.0;
  const auto path = model.stopping_path(Pose4{}, v, m.latency_s, 0.02, 0.4);
  ASSERT_GT(path.size(), 10u);
  EXPECT_NEAR(path.back().position.x(), m.stopping_distance_m(caps::Axis::X, 1.0), 0.01);
  for (size_t i = 1; i < path.size(); ++i) {
    EXPECT_LE((path[i].position - path[i - 1].position).norm(), 0.02 + 1e-9);
  }
}

TEST(Motion, StoppingPathOfARotationStaysInPlace) {
  const MotionModel model(differential());
  Twist v;
  v.yaw_rate = 1.0;
  const auto path = model.stopping_path(Pose4{}, v, 0.1, 0.02, 0.4);
  EXPECT_NEAR(path.back().position.norm(), 0.0, 1e-12);
  EXPECT_NEAR(path.back().yaw, 0.1 + 1.0 / 6.0, 1e-6);
}

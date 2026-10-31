#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

#include "argus/argo/config.hpp"
#include "argus/argo/runtime.hpp"

namespace argo = argus::argo;
namespace caps = argus::capabilities;
namespace fs = std::filesystem;

namespace {

caps::Profile example(const std::string& name) {
  return caps::load_profile_file(std::string(ARGUS_EXAMPLES_DIR) + "/" + name + ".yaml");
}

struct FakeDriver : argo::Driver {
  argo::Inbox* inbox = nullptr;
  std::mutex mutex;
  std::vector<argo::Command> sent;
  bool stopped = false;

  void start(argo::Inbox& in) override { inbox = &in; }
  void send(const argo::Command& c) override {
    std::lock_guard<std::mutex> lock(mutex);
    sent.push_back(c);
  }
  void stop() override { stopped = true; }
};

argo::Odometry odom_at(double t) {
  argo::Odometry o;
  o.stamp = t;
  return o;
}

void quiet(const std::string&) {}

}  // namespace

TEST(Inbox, KeepsOdometryInOrderAndTheNewestFramePerSensor) {
  argo::Inbox inbox;
  inbox.push(odom_at(1.0));
  inbox.push(odom_at(2.0));
  argo::RangeFrame a{1.0, "front", {1.0f}};
  argo::RangeFrame b{2.0, "front", {2.0f}};
  argo::RangeFrame old{0.5, "front", {0.5f}};
  inbox.push(a);
  inbox.push(b);
  inbox.push(old);
  inbox.push(argo::RangeFrame{1.5, "rear", {3.0f}});
  std::vector<argo::Odometry> odom;
  std::vector<argo::RangeFrame> frames;
  inbox.take(odom, frames);
  ASSERT_EQ(odom.size(), 2u);
  EXPECT_EQ(odom[0].stamp, 1.0);
  EXPECT_EQ(odom[1].stamp, 2.0);
  ASSERT_EQ(frames.size(), 2u);
  for (const auto& f : frames) {
    if (f.sensor_id == "front") EXPECT_EQ(f.stamp, 2.0);
  }
  inbox.take(odom, frames);
  EXPECT_TRUE(odom.empty());
  EXPECT_TRUE(frames.empty());
}

TEST(Runtime, EveryTickSendsACommandAndWithNoPlanItIsAStop) {
  FakeDriver driver;
  double now = 100.0;
  argo::Runtime rt(example("planar_differential"), {}, driver, [&] { return now; }, quiet);
  for (int i = 0; i < 10; ++i) {
    rt.core().on_odometry(odom_at(now));
    rt.step(now);
    now += rt.period_s();
  }
  ASSERT_EQ(driver.sent.size(), 10u);
  for (const auto& c : driver.sent) EXPECT_TRUE(c.stop);
}

TEST(Runtime, RunKeepsItsRateAndLeavesTheRobotStopped) {
  FakeDriver driver;
  argo::RuntimeConfig config;
  config.stop_repeat_s = 0.1;
  argo::Runtime rt(example("planar_differential"), config, driver, argo::Runtime::wall_clock, quiet);
  std::thread loop([&] { rt.run(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  rt.stop();
  loop.join();
  const double expected = 0.5 / rt.period_s();
  EXPECT_GT(rt.stats().ticks, 0.8 * expected);
  EXPECT_LT(rt.stats().ticks, 1.2 * expected);
  EXPECT_TRUE(driver.stopped);
  ASSERT_FALSE(driver.sent.empty());
  EXPECT_TRUE(driver.sent.back().stop);
}

TEST(Runtime, AnUnreachableHubNeverHoldsUpATick) {
  FakeDriver driver;
  argo::RuntimeConfig config;
  config.stop_repeat_s = 0.0;
  config.session.node_id = "robot-01";
  config.link.target = "127.0.0.1:1";  // nothing listens here
  config.link.connect_timeout_s = 0.2;
  config.link.reconnect_period_s = 0.1;
  argo::Runtime rt(example("planar_differential"), config, driver, argo::Runtime::wall_clock, quiet);
  std::thread loop([&] { rt.run(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  rt.stop();
  loop.join();
  EXPECT_GT(rt.stats().ticks, 0.8 * 0.8 / rt.period_s());
  EXPECT_LT(rt.stats().worst_s, rt.period_s());
}

TEST(Config, ReadsTheFileAndNamesWhatIsMissing) {
  const fs::path dir = fs::temp_directory_path() / "argo_config_test";
  fs::create_directories(dir);
  fs::copy_file(std::string(ARGUS_EXAMPLES_DIR) + "/planar_differential.yaml", dir / "profile.yaml",
                fs::copy_options::overwrite_existing);
  {
    std::ofstream f(dir / "argo.yaml");
    f << "node_id: robot-01\nprofile: profile.yaml\nprofile_hash: sha256:abc\n";
  }
  const argo::ArgoFiles files = argo::load_argo_config((dir / "argo.yaml").string());
  EXPECT_EQ(files.runtime.session.node_id, "robot-01");
  EXPECT_EQ(files.runtime.session.profile_hash, "sha256:abc");
  EXPECT_TRUE(files.runtime.link.target.empty());

  {
    std::ofstream f(dir / "argo.yaml");
    f << "profile: profile.yaml\nprofile_hash: sha256:abc\nhub:\n  target: 127.0.0.1:7443\n";
  }
  try {
    argo::load_argo_config((dir / "argo.yaml").string());
    FAIL() << "expected an error";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("node_id"), std::string::npos);
  }
  {
    std::ofstream f(dir / "argo.yaml");
    f << "node_id: robot-01\nprofile: profile.yaml\nprofile_hash: sha256:abc\nhub:\n  target: 127.0.0.1:7443\n";
  }
  try {
    argo::load_argo_config((dir / "argo.yaml").string());
    FAIL() << "expected an error";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("hub.ca"), std::string::npos) << e.what();
  }
  fs::remove_all(dir);
}

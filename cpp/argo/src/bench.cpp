// argo_bench: how long Argo's core takes on this machine, against the budgets it has to meet.
//
//   argo_bench PROFILE [--seconds S]
//
// Drives the body in simulated time through a room with a wall and pillars, ray casting its
// range sensors at their declared rates, and times every call into the core with the wall
// clock: each range frame (map update and view marking), each control tick (safety
// supervisor), and each plan. The planner runs here inside the tick, deterministically; on the
// robot it has a thread of its own (NodeConfig::plan_in_tick), so ticks that planned are timed
// as plans. Budgets: a control tick within half the control period at the 99th percentile and
// at worst within the period itself; a plan within half the planner period at the 99th
// percentile and at worst within the period; each sensor's frames within half its frame
// period at the 99th percentile; and all of it under half of one core. Exits 1 when a budget
// is missed.
//
// Each stage inside a frame and a tick is timed too (Node::stage_times), against a share of
// its parent's budget (the frame budget of the fastest sensor, or the tick budget): what
// every feature costs shows, and one that grows too heavy fails here. The link's work is
// timed as Argo's session does it (evidence taken from the map and encoded every 0.5 s),
// and kept under its own share of a core so planning never starves it.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "argus/capabilities/capabilities.hpp"
#include "argus/capabilities/profile.hpp"
#include "argus/node/node.hpp"
#include "argus/node/session.hpp"

namespace {

using argus::node::Vec3;

struct Box {
  Vec3 lo;
  Vec3 hi;
};

// A 12 m room, a wall across the middle with a 2 m gap, and four pillars.
std::vector<Box> world() {
  std::vector<Box> w;
  const double h = 2.5;
  w.push_back({Vec3(-6.2, -6.2, 0.0), Vec3(6.2, -6.0, h)});
  w.push_back({Vec3(-6.2, 6.0, 0.0), Vec3(6.2, 6.2, h)});
  w.push_back({Vec3(-6.2, -6.0, 0.0), Vec3(-6.0, 6.0, h)});
  w.push_back({Vec3(6.0, -6.0, 0.0), Vec3(6.2, 6.0, h)});
  w.push_back({Vec3(2.0, -6.0, 0.0), Vec3(2.3, 0.5, h)});
  w.push_back({Vec3(2.0, 2.5, 0.0), Vec3(2.3, 6.0, h)});
  for (double x : {-3.0, -1.0}) {
    for (double y : {-2.0, 2.0}) {
      w.push_back({Vec3(x - 0.15, y - 0.15, 0.0), Vec3(x + 0.15, y + 0.15, h)});
    }
  }
  return w;
}

// Distance along a unit direction to the first box or the floor; infinity beyond `max_range`.
double cast(const std::vector<Box>& boxes, const Vec3& o, const Vec3& d, double max_range) {
  double best = max_range;
  if (d.z() < -1e-9) {
    best = std::min(best, -o.z() / d.z());
  }
  for (const Box& b : boxes) {
    double t0 = 0.0;
    double t1 = best;
    bool hit = true;
    for (int a = 0; a < 3 && hit; ++a) {
      if (std::abs(d[a]) < 1e-12) {
        hit = o[a] >= b.lo[a] && o[a] <= b.hi[a];
        continue;
      }
      double n = (b.lo[a] - o[a]) / d[a];
      double f = (b.hi[a] - o[a]) / d[a];
      if (n > f) {
        std::swap(n, f);
      }
      t0 = std::max(t0, n);
      t1 = std::min(t1, f);
      hit = t0 <= t1;
    }
    if (hit && t0 > 1e-6) {
      best = std::min(best, t0);
    }
  }
  return best < max_range ? best : std::numeric_limits<double>::infinity();
}

struct Stats {
  std::vector<double> ms;
  double total_ms = 0.0;

  void add(double v) {
    ms.push_back(v);
    total_ms += v;
  }
  double pct(double p) const {
    if (ms.empty()) {
      return 0.0;
    }
    std::vector<double> s = ms;
    const size_t k = std::min(s.size() - 1, static_cast<size_t>(p * (s.size() - 1)));
    std::nth_element(s.begin(), s.begin() + k, s.end());
    return s[k];
  }
  double max() const { return ms.empty() ? 0.0 : *std::max_element(ms.begin(), ms.end()); }
};

double now_ms() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double, std::milli>(clock::now().time_since_epoch()).count();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: argo_bench PROFILE [--seconds S]\n");
    return 2;
  }
  double seconds = 30.0;
  for (int i = 2; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--seconds") {
      seconds = std::atof(argv[i + 1]);
    }
  }
  argus::capabilities::Profile profile;
  try {
    profile = argus::capabilities::load_profile_file(argv[1]);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 2;
  }
  argus::node::NodeConfig config;
  config.map.record_evidence = true;
  argus::node::Node node(profile, config);
  node.stage_times().enabled = true;
  argus::node::EvidenceOutbox outbox(1, 400000, 1u << 30);
  Stats link;
  double next_evidence = 0.0;
  const auto& prof = node.profile();
  const bool aerial = node.capabilities().motion.space == argus::capabilities::MotionSpace::Aerial;
  const double period = node.control_period_s();
  const std::vector<Box> boxes = world();

  struct Sensor {
    std::string id;
    Eigen::Isometry3d mount;
    double period;
    double max_range;
    double next = 0.0;
    Stats stats;
  };
  std::vector<Sensor> sensors;
  for (const std::string& id : node.range_sensor_ids()) {
    const auto& spec = *std::find_if(prof.sensors.begin(), prof.sensors.end(), [&](const auto& s) { return s.id == id; });
    Sensor s;
    s.id = id;
    s.mount = prof.geometry.frames.at(spec.frame).transform();
    s.period = 1.0 / spec.rate_hz;
    s.max_range = spec.range_m->max;
    sensors.push_back(std::move(s));
  }

  argus::node::Pose4 pose;
  pose.position = Vec3(-5.0, 0.0, aerial ? 1.5 : 0.0);
  argus::node::Plan plan;
  plan.generation = 1;
  plan.expires = std::numeric_limits<double>::infinity();
  plan.waypoints.push_back({"far", Vec3(5.0, 1.5, pose.position.z()), 0.4});

  Stats tick;
  Stats planning;
  argus::node::Twist vel;
  bool planned = false;
  for (double t = 0.0; t < seconds; t += period) {
    argus::node::Odometry odom;
    odom.stamp = t;
    odom.pose = pose;
    odom.twist = vel;
    node.on_odometry(odom);
    for (Sensor& s : sensors) {
      if (t + 1e-9 < s.next) {
        continue;
      }
      s.next = t + s.period;
      const auto& pattern = node.ray_pattern(s.id);
      const Eigen::Isometry3d to_odom = pose.transform() * s.mount;
      argus::node::RangeFrame frame;
      frame.stamp = t;
      frame.sensor_id = s.id;
      frame.ranges.reserve(pattern.size());
      for (const Vec3& d : pattern.directions) {
        frame.ranges.push_back(static_cast<float>(cast(boxes, to_odom.translation(), to_odom.linear() * d, s.max_range)));
      }
      const double a = now_ms();
      node.on_range_frame(frame);
      s.stats.add(now_ms() - a);
    }
    if (t + 1e-9 >= next_evidence) {
      next_evidence = t + 0.5;
      const double a = now_ms();
      outbox.add(node.take_evidence());
      while (const auto batch = outbox.next(1u << 20, t)) {
        outbox.ack(batch->seq);
      }
      link.add(now_ms() - a);
    }
    if (!planned) {
      planned = node.set_plan(plan, t);
    }
    const double a = now_ms();
    const argus::node::TickResult r = node.tick(t);
    (node.last_tick_planned() ? planning : tick).add(now_ms() - a);
    // Follow the command as a perfect plant would.
    vel = r.command.stop ? argus::node::Twist{} : r.command.velocity;
    if (aerial) {
      vel.linear = pose.transform().linear().transpose() * r.command.setpoint_velocity;
      vel.yaw_rate = r.command.setpoint_yaw_rate;
    }
    pose.position += pose.rotate(vel.linear) * period;
    pose.yaw += vel.yaw_rate * period;
  }

  bool ok = true;
  auto line = [&](const char* name, const Stats& s, double budget_ms, double hard_ms) {
    const bool pass = s.pct(0.99) <= budget_ms && s.max() <= hard_ms;
    ok = ok && pass;
    std::printf("%-22s calls %6zu  p50 %7.3f  p99 %7.3f  max %7.3f ms   budget p99 %6.2f max %6.2f  %s\n", name,
                s.ms.size(), s.pct(0.5), s.pct(0.99), s.max(), budget_ms, hard_ms, pass ? "ok" : "MISSED");
  };
  std::printf("%s: %.0f s simulated, control %.0f Hz\n", prof.id.c_str(), seconds, 1.0 / period);
  line("tick", tick, 500.0 * period, 1000.0 * period);
  const double plan_period = argus::node::NodeConfig{}.planner.period_s;
  line("plan", planning, 500.0 * plan_period, 1000.0 * plan_period);
  double busy = tick.total_ms + planning.total_ms;
  for (const Sensor& s : sensors) {
    line(("frame " + s.id).c_str(), s.stats, 500.0 * s.period, 1000.0 * s.period);
    busy += s.stats.total_ms;
  }
  // Stages, against a share of their parent's budget: each sensor's frame budget, or the tick's.
  auto& stages = node.stage_times().ms;
  auto stage = [&](const std::string& name, double frac, double parent) {
    Stats st;
    for (double v : stages[name]) {
      st.add(v);
    }
    line(("  " + name).c_str(), st, frac * parent, 2.0 * frac * parent);
  };
  const std::pair<const char*, double> frame_shares[] = {
      {"frame health", 0.05},         {"frame rays", 0.25}, {"frame drop edges", 0.15}, {"frame tracking", 0.25},
      {"frame odometry alarm", 0.25}, {"frame map", 0.5},   {"frame view", 0.5},
  };
  for (const Sensor& s : sensors) {
    for (const auto& [name, frac] : frame_shares) {
      stage(std::string(name) + " " + s.id, frac, 500.0 * s.period);
    }
  }
  stage("tick movers", 0.25, 500.0 * period);
  stage("tick supervisor", 0.5, 500.0 * period);
  // The link has threads of its own on the robot: each round must finish well within the
  // evidence period, and the whole within its share of a core (below).
  line("link evidence", link, 250.0, 500.0);
  busy += link.total_ms;
  const double link_share = link.total_ms / (1000.0 * seconds);
  const bool link_ok = link_share <= 0.1;
  ok = ok && link_ok;
  std::printf("%-22s %.1f%% of one core (budget 10%%)  %s\n", "link total", 100.0 * link_share, link_ok ? "ok" : "MISSED");
  const double share = busy / (1000.0 * seconds);
  const bool cpu_ok = share <= 0.5;
  ok = ok && cpu_ok;
  std::printf("%-22s %.1f%% of one core (budget 50%%)  %s\n", "core total", 100.0 * share, cpu_ok ? "ok" : "MISSED");
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}

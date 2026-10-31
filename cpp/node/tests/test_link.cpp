#include <gtest/gtest.h>

#include "argus/node/link.hpp"

using namespace argus::node;

namespace {

LinkConfig config() {
  LinkConfig c;
  c.lost_after_s = 3.0;
  c.heartbeat_period_s = 0.2;
  c.recover_after_s = 1.0;
  return c;
}

// Runs heartbeats both ways from `from` to `to`, with the given one-way delay and every
// `drop_every`-th echo lost (0 for none).
void exchange(LinkMonitor& m, uint64_t& seq, double from, double to, double delay, int drop_every = 0) {
  for (double t = from; t < to - 1e-9; t += 0.2) {
    m.on_sent(++seq, t);
    if (drop_every == 0 || seq % drop_every != 0) {
      m.on_heartbeat(seq, t + 2 * delay);
    }
    m.update(t + 2 * delay);
  }
}

}  // namespace

TEST(Link, LostBeforeAnySession) {
  LinkMonitor m(config());
  EXPECT_EQ(m.update(100.0), LinkState::Lost);
  m.on_heartbeat(1, 100.0);
  EXPECT_EQ(m.update(100.1), LinkState::Lost);
}

TEST(Link, ConnectsAfterRecoveryTime) {
  LinkMonitor m(config());
  m.on_session_start(0.0);
  uint64_t seq = 0;
  exchange(m, seq, 0.0, 0.8, 0.01);
  EXPECT_EQ(m.state(), LinkState::Lost);
  exchange(m, seq, 0.8, 2.0, 0.01);
  EXPECT_EQ(m.state(), LinkState::Connected);
  EXPECT_NEAR(m.round_trip_s(), 0.02, 1e-9);
}

TEST(Link, SlowOrLossyLinkIsDegraded) {
  LinkMonitor slow(config());
  slow.on_session_start(0.0);
  uint64_t seq = 0;
  exchange(slow, seq, 0.0, 4.0, 0.3);
  EXPECT_EQ(slow.state(), LinkState::Degraded);

  LinkMonitor lossy(config());
  lossy.on_session_start(0.0);
  seq = 0;
  exchange(lossy, seq, 0.0, 6.0, 0.01, 3);
  EXPECT_GT(lossy.loss_fraction(), 0.2);
  EXPECT_EQ(lossy.state(), LinkState::Degraded);
}

TEST(Link, SilenceLosesTheLinkOnlyAfterTheProfileTimeout) {
  LinkMonitor m(config());
  m.on_session_start(0.0);
  uint64_t seq = 0;
  exchange(m, seq, 0.0, 2.0, 0.01);
  ASSERT_EQ(m.state(), LinkState::Connected);
  const double last = 1.8 + 0.02;
  EXPECT_EQ(m.update(last + 1.0), LinkState::Degraded);
  EXPECT_EQ(m.update(last + 2.9), LinkState::Degraded);
  EXPECT_EQ(m.update(last + 3.01), LinkState::Lost);
  // Heartbeats again: still lost until they have flowed for the recovery time.
  m.on_session_start(10.0);
  exchange(m, seq, 10.0, 10.6, 0.01);
  EXPECT_EQ(m.state(), LinkState::Lost);
  exchange(m, seq, 10.6, 11.6, 0.01);
  EXPECT_EQ(m.state(), LinkState::Connected);
}

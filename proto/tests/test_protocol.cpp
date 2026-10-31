#include <gtest/gtest.h>

#include "argus/protocol/v1/link.pb.h"

using namespace argus::protocol::v1;

TEST(Protocol, MissionPlanRoundTrip) {
  HubMessage out;
  out.set_seq(7);
  MissionPlan* plan = out.mutable_mission_plan();
  plan->set_generation(3);
  plan->mutable_expires()->set_unix_nanos(1'700'000'000'000'000'000);
  Waypoint* wp = plan->add_waypoints();
  wp->set_id("wp-1");
  wp->mutable_pose()->mutable_position()->set_x(4.5);
  wp->set_speed_cap_mps(0.6);
  wp->set_speculative(true);

  HubMessage in;
  ASSERT_TRUE(in.ParseFromString(out.SerializeAsString()));
  ASSERT_EQ(in.body_case(), HubMessage::kMissionPlan);
  EXPECT_EQ(in.mission_plan().generation(), 3u);
  EXPECT_EQ(in.mission_plan().waypoints(0).id(), "wp-1");
  EXPECT_DOUBLE_EQ(in.mission_plan().waypoints(0).pose().position().x(), 4.5);
  EXPECT_TRUE(in.mission_plan().waypoints(0).speculative());
}

TEST(Protocol, StepFailureCarriesCause) {
  NodeMessage out;
  Event* event = out.mutable_event();
  event->set_kind(EVENT_KIND_STEP_FAILED);
  event->set_cause(FAILURE_CAUSE_COMPUTE_DEADLINE_MISS);

  NodeMessage in;
  ASSERT_TRUE(in.ParseFromString(out.SerializeAsString()));
  EXPECT_EQ(in.event().cause(), FAILURE_CAUSE_COMPUTE_DEADLINE_MISS);
}

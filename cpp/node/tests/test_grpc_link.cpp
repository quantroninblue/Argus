#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <grpcpp/grpcpp.h>
#include <gtest/gtest.h>

#include "argus/capabilities/profile.hpp"
#include "argus/node/grpc_link.hpp"
#include "argus/node/session.hpp"
#include "argus/protocol/v1/link.grpc.pb.h"

namespace pb = argus::protocol::v1;
using namespace argus::node;

namespace {

// Just enough hub for a session: accepts every Hello and answers each heartbeat.
class TinyHub final : public pb::HubLink::Service {
 public:
  grpc::Status Session(grpc::ServerContext*, grpc::ServerReaderWriter<pb::HubMessage, pb::NodeMessage>* stream) override {
    pb::NodeMessage in;
    uint64_t seq = 0;
    while (stream->Read(&in)) {
      pb::HubMessage out;
      out.set_seq(++seq);
      if (in.has_hello()) {
        out.mutable_session_accept()->set_session_id("s-" + in.hello().node_id());
        out.mutable_session_accept()->set_map_epoch(1);
      } else if (in.has_heartbeat()) {
        out.mutable_heartbeat()->set_seq(seq);
        out.mutable_heartbeat()->set_echo_seq(in.heartbeat().seq());
      } else {
        continue;
      }
      stream->Write(out);
    }
    return grpc::Status::OK;
  }
};

struct Server {
  TinyHub service;
  std::unique_ptr<grpc::Server> server;

  explicit Server(int& port) {
    grpc::ServerBuilder builder;
    builder.AddListeningPort("127.0.0.1:" + std::to_string(port), grpc::InsecureServerCredentials(), &port);
    builder.RegisterService(&service);
    server = builder.BuildAndStart();
  }
  ~Server() { server->Shutdown(std::chrono::system_clock::now()); }
};

double seconds_since(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

// Runs Argo's control loop against the link until `done` or the time runs out.
template <typename Done>
bool drive(Node& node, Session& session, GrpcLink& link, std::chrono::steady_clock::time_point t0,
           double for_s, Done done) {
  const double until = seconds_since(t0) + for_s;
  while (seconds_since(t0) < until) {
    const double now = seconds_since(t0);
    GrpcLink::Update u = link.poll();
    size_t next = 0;
    auto receive_until = [&](size_t end) {
      for (; next < end; ++next) session.receive(u.received[next], now);
    };
    for (const auto& t : u.transitions) {
      receive_until(t.after);
      t.connected ? session.on_connected(now) : session.on_disconnected(now);
    }
    receive_until(u.received.size());
    Odometry o;
    o.stamp = now;
    node.on_odometry(o);
    session.step(now);
    link.send(session.take_outgoing());
    if (done()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

}  // namespace

TEST(GrpcLink, CarriesAnArgoSessionAndReconnectsAfterTheHubRestarts) {
  int port = 0;
  auto server = std::make_unique<Server>(port);
  ASSERT_GT(port, 0);

  Node node(argus::capabilities::load_profile_file(std::string(ARGUS_EXAMPLES_DIR) + "/planar_omnidirectional.yaml"));
  SessionConfig config;
  config.node_id = "argo-test";
  Session session(node, config);
  GrpcLinkConfig lc;
  lc.target = "127.0.0.1:" + std::to_string(port);
  lc.reconnect_period_s = 0.2;
  GrpcLink link(lc);
  const auto t0 = std::chrono::steady_clock::now();

  ASSERT_TRUE(drive(node, session, link, t0, 5.0, [&] { return session.link_state() == LinkState::Connected; }));
  EXPECT_TRUE(session.accepted());
  EXPECT_LT(session.link().round_trip_s(), 0.2);

  server.reset();
  ASSERT_TRUE(drive(node, session, link, t0, 5.0, [&] { return !session.connected(); }));
  EXPECT_FALSE(link.connected());

  server = std::make_unique<Server>(port);
  ASSERT_TRUE(drive(node, session, link, t0, 5.0, [&] { return session.accepted(); }));
  EXPECT_EQ(link.streams_opened(), 2);
}

TEST(GrpcLink, NoHubMeansNoStreamAndNoHang) {
  GrpcLinkConfig lc;
  lc.target = "127.0.0.1:1";  // nothing listens here
  lc.connect_timeout_s = 0.2;
  const auto t0 = std::chrono::steady_clock::now();
  {
    GrpcLink link(lc);
    link.send({"ignored"});
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    EXPECT_FALSE(link.connected());
    EXPECT_TRUE(link.poll().transitions.empty());
  }
  // Shutting down waits at most for one connection attempt.
  EXPECT_LT(seconds_since(t0), 1.5);
}

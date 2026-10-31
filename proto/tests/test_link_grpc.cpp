#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>
#include <gtest/gtest.h>

#include "argus/protocol/v1/link.grpc.pb.h"

using namespace argus::protocol::v1;

namespace {

// Answers Hello with SessionAccept and echoes heartbeats, which is enough to show messages
// cross a real stream in both directions.
class EchoHub final : public HubLink::Service {
 public:
  grpc::Status Session(grpc::ServerContext*,
                       grpc::ServerReaderWriter<HubMessage, NodeMessage>* stream) override {
    NodeMessage in;
    uint64_t seq = 0;
    while (stream->Read(&in)) {
      HubMessage out;
      out.set_seq(++seq);
      if (in.has_hello()) {
        out.mutable_session_accept()->set_session_id("s-" + in.hello().node_id());
      } else if (in.has_heartbeat()) {
        *out.mutable_heartbeat() = in.heartbeat();
      } else {
        continue;
      }
      stream->Write(out);
    }
    return grpc::Status::OK;
  }

  grpc::Status Sensors(grpc::ServerContext*, grpc::ServerReader<SensorFrame>* reader,
                       SensorAck* ack) override {
    SensorFrame frame;
    uint64_t count = 0;
    while (reader->Read(&frame)) ++count;
    ack->set_frames_received(count);
    return grpc::Status::OK;
  }
};

struct Running {
  EchoHub service;
  std::unique_ptr<grpc::Server> server;
  std::unique_ptr<HubLink::Stub> stub;

  Running() {
    int port = 0;
    grpc::ServerBuilder builder;
    builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
    builder.RegisterService(&service);
    server = builder.BuildAndStart();
    stub = HubLink::NewStub(grpc::CreateChannel("127.0.0.1:" + std::to_string(port),
                                                grpc::InsecureChannelCredentials()));
  }
  ~Running() { server->Shutdown(); }
};

}  // namespace

TEST(LinkGrpc, SessionStreamCarriesBothDirections) {
  Running hub;
  grpc::ClientContext context;
  auto stream = hub.stub->Session(&context);

  NodeMessage hello;
  hello.set_seq(1);
  hello.mutable_hello()->set_node_id("argo-1");
  ASSERT_TRUE(stream->Write(hello));
  HubMessage reply;
  ASSERT_TRUE(stream->Read(&reply));
  ASSERT_EQ(reply.body_case(), HubMessage::kSessionAccept);
  EXPECT_EQ(reply.session_accept().session_id(), "s-argo-1");

  NodeMessage beat;
  beat.set_seq(2);
  beat.mutable_heartbeat()->set_seq(9);
  ASSERT_TRUE(stream->Write(beat));
  ASSERT_TRUE(stream->Read(&reply));
  ASSERT_EQ(reply.body_case(), HubMessage::kHeartbeat);
  EXPECT_EQ(reply.heartbeat().seq(), 9u);

  stream->WritesDone();
  EXPECT_TRUE(stream->Finish().ok());
}

TEST(LinkGrpc, SensorStreamIsSeparate) {
  Running hub;
  grpc::ClientContext context;
  SensorAck ack;
  auto writer = hub.stub->Sensors(&context, &ack);
  for (int i = 0; i < 5; ++i) {
    SensorFrame frame;
    frame.set_sensor_id("front");
    ASSERT_TRUE(writer->Write(frame));
  }
  writer->WritesDone();
  ASSERT_TRUE(writer->Finish().ok());
  EXPECT_EQ(ack.frames_received(), 5u);
}

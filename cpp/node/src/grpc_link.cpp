#include "argus/node/grpc_link.hpp"

#include <chrono>

#include <grpcpp/grpcpp.h>

#include "argus/protocol/v1/link.grpc.pb.h"

namespace argus::node {

namespace pb = argus::protocol::v1;

namespace {

std::shared_ptr<grpc::ChannelCredentials> credentials(const GrpcLinkConfig& c) {
  if (c.ca_pem.empty() && c.cert_pem.empty() && c.key_pem.empty()) {
    return grpc::InsecureChannelCredentials();
  }
  grpc::SslCredentialsOptions ssl;
  ssl.pem_root_certs = c.ca_pem;
  ssl.pem_cert_chain = c.cert_pem;
  ssl.pem_private_key = c.key_pem;
  return grpc::SslCredentials(ssl);
}

std::chrono::system_clock::time_point after(double seconds) {
  return std::chrono::system_clock::now() +
         std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::duration<double>(seconds));
}

}  // namespace

struct GrpcLink::Stream {
  grpc::ClientContext context;
  std::unique_ptr<grpc::ClientReaderWriter<pb::NodeMessage, pb::HubMessage>> call;
};

GrpcLink::GrpcLink(GrpcLinkConfig config) : config_(std::move(config)) {
  thread_ = std::thread([this] { run(); });
}

GrpcLink::~GrpcLink() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    if (stream_) {
      stream_->context.TryCancel();
    }
  }
  wake_.notify_all();
  thread_.join();
}

void GrpcLink::send(const std::vector<std::string>& messages) {
  if (messages.empty()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_) {
      return;
    }
    outgoing_.insert(outgoing_.end(), messages.begin(), messages.end());
  }
  wake_.notify_all();
}

GrpcLink::Update GrpcLink::poll() {
  std::lock_guard<std::mutex> lock(mutex_);
  Update out;
  std::swap(out, pending_);
  return out;
}

void GrpcLink::set_connected(bool connected) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (connected_ == connected) {
    return;
  }
  connected_ = connected;
  pending_.transitions.push_back({connected, pending_.received.size()});
  if (!connected) {
    outgoing_.clear();
  }
}

void GrpcLink::run() {
  grpc::ChannelArguments args;
  if (!config_.server_name.empty()) {
    args.SetSslTargetNameOverride(config_.server_name);
  }
  // Notice a dead peer even when nothing is being sent.
  args.SetInt(GRPC_ARG_KEEPALIVE_TIME_MS, 2000);
  args.SetInt(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 2000);
  args.SetInt(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);
  // Reconnect on our own schedule rather than gRPC's growing backoff.
  args.SetInt(GRPC_ARG_INITIAL_RECONNECT_BACKOFF_MS, static_cast<int>(config_.reconnect_period_s * 1000));
  args.SetInt(GRPC_ARG_MAX_RECONNECT_BACKOFF_MS, static_cast<int>(config_.reconnect_period_s * 1000));

  while (!stop_) {
    auto channel = grpc::CreateCustomChannel(config_.target, credentials(config_), args);
    if (!channel->WaitForConnected(after(config_.connect_timeout_s))) {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait_for(lock, std::chrono::duration<double>(config_.reconnect_period_s), [this] { return stop_.load(); });
      continue;
    }
    auto stub = pb::HubLink::NewStub(channel);
    auto stream = std::make_shared<Stream>();
    stream->call = stub->Session(&stream->context);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_) {
        stream->context.TryCancel();
      }
      stream_ = stream;
    }
    ++streams_opened_;
    set_connected(true);

    std::atomic<bool> open{true};
    std::thread reader([this, stream, &open] {
      pb::HubMessage m;
      while (stream->call->Read(&m)) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.received.push_back(m.SerializeAsString());
      }
      open = false;
      wake_.notify_all();
    });

    while (!stop_ && open) {
      std::deque<std::string> batch;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_for(lock, std::chrono::milliseconds(50),
                       [&] { return stop_ || !open || !outgoing_.empty(); });
        batch.swap(outgoing_);
      }
      for (const std::string& bytes : batch) {
        pb::NodeMessage m;
        if (m.ParseFromString(bytes) && !stream->call->Write(m)) {
          open = false;
          break;
        }
      }
    }

    set_connected(false);
    stream->context.TryCancel();
    reader.join();
    stream->call->Finish();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stream_.reset();
    }
    if (!stop_) {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait_for(lock, std::chrono::duration<double>(config_.reconnect_period_s), [this] { return stop_.load(); });
    }
  }
}

}  // namespace argus::node

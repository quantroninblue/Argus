#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace argus::node {

struct GrpcLinkConfig {
  // host:port of the hub (or of anything in between, such as a proxy).
  std::string target;
  // PEM. With all three set the link uses mutual TLS: Argo checks the hub against `ca` and
  // presents its own certificate, whose common name must be its node id. Empty means an
  // unencrypted channel, for local tests only.
  std::string ca_pem;
  std::string cert_pem;
  std::string key_pem;
  // Name to expect in the hub's certificate when `target` is an address it does not list.
  std::string server_name;
  double connect_timeout_s = 2.0;
  double reconnect_period_s = 1.0;
};

// The transport under Argo's Session: one gRPC Session stream to the hub, opened by Argo and
// reopened when it drops. It only moves bytes. The control loop calls send() with what the
// session produced and poll() for what arrived; the stream runs on its own threads so a
// stalled network can never hold up a control tick.
class GrpcLink {
 public:
  struct Transition {
    bool connected = false;
    // How many of `received` arrived before this change. A hub that rejects a session sends
    // its reason and closes the stream; the reason has to be handled before the close.
    size_t after = 0;
  };
  struct Update {
    // Each change of the stream's state since the last poll, oldest first.
    std::vector<Transition> transitions;
    std::vector<std::string> received;
  };

  explicit GrpcLink(GrpcLinkConfig config);
  ~GrpcLink();
  GrpcLink(const GrpcLink&) = delete;
  GrpcLink& operator=(const GrpcLink&) = delete;

  // Serialized NodeMessages. Dropped when no stream is open: the session buffers what matters.
  void send(const std::vector<std::string>& messages);
  Update poll();
  bool connected() const { return connected_; }
  int streams_opened() const { return streams_opened_; }

 private:
  struct Stream;

  void run();
  void set_connected(bool connected);

  GrpcLinkConfig config_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> connected_{false};
  std::atomic<int> streams_opened_{0};
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<std::string> outgoing_;
  Update pending_;
  std::shared_ptr<Stream> stream_;
  std::thread thread_;
};

}  // namespace argus::node

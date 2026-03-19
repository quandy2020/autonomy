#include "autonomy/autoviz/core/server/foxglove_server.hpp"

namespace autoviz {
namespace server {

FoxgloveServer::FoxgloveServer(const config::FoxgloveConfig& cfg)
    : cfg_(cfg) {}

FoxgloveServer::~FoxgloveServer() {
  Stop();
}

void FoxgloveServer::Start() {
  // TODO: Wire up to autoviz_foxglove_base and start the real WebSocket server.
  running_.store(true);
}

void FoxgloveServer::Stop() {
  if (!running_.exchange(false)) {
    return;
  }
  // TODO: Stop underlying server instance and release resources.
}

void FoxgloveServer::RegisterSchema(
    const converter::FoxgloveMessageSchema& schema) {
  std::lock_guard<std::mutex> lock(mutex_);
  schemas_by_proto_type_[schema.proto_full_name] = schema;
}

void FoxgloveServer::EnsureChannel(const std::string& topic,
                                   const std::string& msg_type,
                                   const std::string& encoding) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string key = topic + "|" + msg_type + "|" + encoding;
  registered_channels_.insert(key);
}

void FoxgloveServer::PublishRaw(const std::string& topic,
                                const std::string& msg_type,
                                const std::string& encoding,
                                const void* data,
                                std::size_t size,
                                std::uint64_t timestamp_ns) {
  if (!running_.load()) {
    return;
  }

  (void)topic;
  (void)msg_type;
  (void)encoding;
  (void)data;
  (void)size;
  (void)timestamp_ns;

  EnsureChannel(topic, msg_type, encoding);

  // TODO: Use underlying foxglove server (from autoviz_foxglove_base)
  // to publish this message on the appropriate channel.
}

}  // namespace server
}  // namespace autoviz


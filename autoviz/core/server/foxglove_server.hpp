// Thin wrapper around the underlying foxglove websocket server implementation.
#pragma once

#include <cstddef>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "autonomy/autoviz/core/config/config.hpp"
#include "autonomy/autoviz/core/converter/schema_converter.hpp"

namespace autoviz {
namespace server {

class FoxgloveServer {
 public:
  explicit FoxgloveServer(const config::FoxgloveConfig& cfg);
  ~FoxgloveServer();

  void Start();
  void Stop();

  bool IsRunning() const { return running_.load(); }

  // Register one schema and all dependencies in local registry.
  void RegisterSchema(const converter::FoxgloveMessageSchema& schema);

  // Ensure topic/channel metadata exists for publishing.
  void EnsureChannel(const std::string& topic,
                     const std::string& msg_type,
                     const std::string& encoding);

  // Publish a raw protobuf message to a foxglove channel.
  // `msg_type` is the full automsgs type name, `encoding` is typically "protobuf".
  void PublishRaw(const std::string& topic,
                  const std::string& msg_type,
                  const std::string& encoding,
                  const void* data,
                  std::size_t size,
                  std::uint64_t timestamp_ns);

 private:
  config::FoxgloveConfig cfg_;
  std::atomic<bool> running_{false};
  std::mutex mutex_;
  std::unordered_map<std::string, converter::FoxgloveMessageSchema> schemas_by_proto_type_;
  std::unordered_set<std::string> registered_channels_;
};

}  // namespace server
}  // namespace autoviz


// Foxglove WebSocket：为每个 topic 建通道并写入 protobuf 或已编码的 Foxglove 消息。
#pragma once

#include <cstddef>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include "autonomy/autoviz/core/settings.hpp"
#include "autonomy/autoviz/core/convert/schema/foxglove_channel_schema.hpp"

// Forward declarations to avoid pulling foxglove SDK headers into this header.
namespace foxglove {
class WebSocketServer;
class RawChannel;
}  // namespace foxglove

namespace autoviz {
namespace converter {
struct FoxgloveConvertedMessage;
}  // namespace converter

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

  void EnsureChannel(const std::string& topic, const std::string& msg_type,
                     const std::string& encoding, const std::string* proto_desc);

  // `msg_type`：automsgs 全名；`encoding` 一般为 "protobuf"。
  void EmitProtobuf(const std::string& topic, const std::string& msg_type,
                    const std::string& encoding, const void* data, std::size_t size,
                    std::uint64_t timestamp_ns, const std::string* proto_desc);

  // 已编码的 Foxglove schema（如 SceneUpdate），用于 `topic/foxglove` 等。
  void EmitFoxglove(const std::string& topic, const converter::FoxgloveConvertedMessage& m,
                    std::uint64_t timestamp_ns);

 private:
  std::string MakeChannelKey(const std::string& topic,
                             const std::string& msg_type,
                             const std::string& encoding) const;

  void PrepareFoxgloveChannel(const std::string& topic, const std::string& schema_name,
                              const std::string& schema_encoding,
                              const std::string& schema_data_blob);

  config::FoxgloveConfig cfg_;
  std::atomic<bool> running_{false};
  std::mutex mutex_;
  std::unordered_map<std::string, converter::FoxgloveMessageSchema> schemas_by_proto_type_;
  std::unordered_map<std::string, std::unique_ptr<foxglove::RawChannel>> channels_;
  // Keep a stable copy of descriptor bytes so foxglove schema pointers remain valid.
  std::unordered_map<std::string, std::string> proto_desc_by_proto_type_;
  /// Foxglove 官方 schema 定义字节（与 channel key 对应），供 `RawChannel::create` 使用。
  std::unordered_map<std::string, std::string> foxglove_native_schema_blob_by_key_;
  std::unique_ptr<foxglove::WebSocketServer> server_;
};

}  // namespace server
}  // namespace autoviz


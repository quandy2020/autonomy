// Bridge between autolink message bus and the foxglove server.
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "autolink/autolink.hpp"
#include "autolink/message/raw_message.hpp"
#include "autonomy/autoviz/core/converter/schema_converter.hpp"

namespace google {
namespace protobuf {
class Message;
class DescriptorPool;
}  // namespace protobuf
}  // namespace google

#include "autonomy/autoviz/core/config/config.hpp"

namespace autoviz {

namespace server {
class FoxgloveServer;
}  // namespace server

namespace bridge {

class AutolinkBridge {
 public:
  AutolinkBridge(const config::AutolinkConfig& cfg, server::FoxgloveServer& server);
  ~AutolinkBridge();

  void Start();
  void Stop();

  // Subscribe to an autolink topic of given message type.
  // Callback is invoked on each received message.
  template <typename MsgT, typename Callback>
  bool Subscribe(const std::string& topic, Callback&& cb);

  bool SubscribeRaw(
      const std::string& topic,
      const std::function<void(const std::string&, const std::shared_ptr<autolink::message::RawMessage>&)>& cb);

  bool EnsureSchemaRegistered(const std::string& msg_type,
                              converter::SchemaConverter& converter,
                              const google::protobuf::DescriptorPool& descriptor_pool);

  // Serialize and forward a protobuf message to foxglove.
  void ForwardToFoxglove(const std::string& topic,
                         const std::string& msg_type,
                         const google::protobuf::Message& msg,
                         std::uint64_t timestamp_ns);

  // Forward already serialized protobuf bytes to foxglove.
  void ForwardRawToFoxglove(const std::string& topic,
                            const std::string& msg_type,
                            const std::string& serialized_msg,
                            std::uint64_t timestamp_ns);

 private:
  config::AutolinkConfig cfg_;
  server::FoxgloveServer& server_;
  std::atomic<bool> running_{false};

  std::shared_ptr<::autolink::Node> node_;
  std::unordered_map<std::string, std::shared_ptr<void>> readers_;
};

template <typename MsgT, typename Callback>
bool AutolinkBridge::Subscribe(const std::string& topic, Callback&& cb) {
  if (!node_) {
    return false;
  }

  auto reader = node_->CreateReader<MsgT>(
      topic,
      [this, topic, cb = std::forward<Callback>(cb)](
          const std::shared_ptr<MsgT>& msg) mutable { cb(topic, msg); });

  if (!reader) {
    return false;
  }
  readers_[topic] = std::move(reader);
  return true;
}

}  // namespace bridge
}  // namespace autoviz


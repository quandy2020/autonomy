// autolink → Foxglove：发现 topic、订阅、转发（含可选 automsgs→Foxglove 镜像）。
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "autolink/autolink.hpp"
#include "autolink/message/raw_message.hpp"
#include "autonomy/autoviz/core/convert/schema/foxglove_channel_schema.hpp"

namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

#include "autonomy/autoviz/core/settings.hpp"

namespace autoviz {

namespace server {
class FoxgloveServer;
}  // namespace server

namespace recorder {
class Recorder;
}  // namespace recorder

namespace autolink {

class AutolinkBridge {
 public:
  AutolinkBridge(const config::AutolinkConfig& cfg, server::FoxgloveServer& server,
                 recorder::Recorder* recorder);
  ~AutolinkBridge();

  void Start();
  void Stop();

  // 订阅 topic，回调收到已解析的 `MsgT`（protobuf）。
  template <typename MsgT, typename Callback>
  bool SubscribeProtobuf(const std::string& topic, Callback&& cb);

  // 订阅 topic，回调收到序列化后的 bytes（RawMessage）。
  bool SubscribeSerialized(const std::string& topic,
      const std::function<void(const std::string&, const std::shared_ptr<::autolink::message::RawMessage>&)>& cb);

  // 将一条 protobuf 消息发到 Foxglove（再序列化）。
  void PublishProtobuf(const std::string& topic, const std::string& msg_type,
                       const google::protobuf::Message& msg, std::uint64_t timestamp_ns);

  // 将已序列化的 protobuf bytes 发到 Foxglove；可选附带 descriptor，并尝试镜像 foxglove schema。
  void PublishSerialized(const std::string& topic, const std::string& msg_type,
                         const std::string& serialized_msg, std::uint64_t timestamp_ns,
                         const std::string* proto_desc);

 private:
  void RunDiscoveryLoop();
  bool TopicMatchesWhitelist(const std::string& topic) const;
  void PollTopicsAndSubscribe();
  void SubscribeToTopic(const std::string& topic_name);

  bool RegisterSchemaForType(const std::string& msg_type);

  config::AutolinkConfig cfg_;
  server::FoxgloveServer& server_;
  recorder::Recorder* recorder_{nullptr};

  std::atomic<bool> running_{false};
  std::atomic<bool> discovery_running_{false};
  std::thread discovery_thread_;

  std::shared_ptr<::autolink::Node> node_;
  std::unordered_map<std::string, std::shared_ptr<void>> readers_;

  converter::SchemaConverter schema_converter_;
  std::set<std::string> subscribed_topics_;
  mutable std::mutex subscribed_mutex_;
};

template <typename MsgT, typename Callback>
bool AutolinkBridge::SubscribeProtobuf(const std::string& topic, Callback&& cb) {
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

}  // namespace autolink
}  // namespace autoviz

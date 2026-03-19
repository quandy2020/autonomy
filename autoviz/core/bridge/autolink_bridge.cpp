#include "autonomy/autoviz/core/bridge/autolink_bridge.hpp"

#include "autonomy/autoviz/core/server/foxglove_server.hpp"
#include "autolink/autolink.hpp"
#include "google/protobuf/message.h"
#include "google/protobuf/descriptor.h"

namespace autoviz {
namespace bridge {

AutolinkBridge::AutolinkBridge(const config::AutolinkConfig& cfg,
                               server::FoxgloveServer& server)
    : cfg_(cfg), server_(server) {}

AutolinkBridge::~AutolinkBridge() {
  Stop();
}

void AutolinkBridge::Start() {
  if (running_.exchange(true)) {
    return;
  }

  if (!node_) {
    node_ = autolink::CreateNode("autoviz_bridge");
  }

  if (!node_) {
    running_.store(false);
  }
}

void AutolinkBridge::Stop() {
  if (!running_.exchange(false)) {
    return;
  }
  readers_.clear();
  node_.reset();
}

void AutolinkBridge::ForwardToFoxglove(const std::string& topic,
                                       const std::string& msg_type,
                                       const google::protobuf::Message& msg,
                                       std::uint64_t timestamp_ns) {
  std::string buffer;
  if (!msg.SerializeToString(&buffer)) {
    return;
  }
  server_.PublishRaw(topic, msg_type, "protobuf", buffer.data(),
                     buffer.size(), timestamp_ns);
}

void AutolinkBridge::ForwardRawToFoxglove(const std::string& topic,
                                          const std::string& msg_type,
                                          const std::string& serialized_msg,
                                          std::uint64_t timestamp_ns) {
  server_.PublishRaw(topic, msg_type, "protobuf", serialized_msg.data(),
                     serialized_msg.size(), timestamp_ns);
}

bool AutolinkBridge::SubscribeRaw(
    const std::string& topic,
    const std::function<void(const std::string&, const std::shared_ptr<autolink::message::RawMessage>&)>& cb) {
  return Subscribe<autolink::message::RawMessage>(topic, cb);
}

bool AutolinkBridge::EnsureSchemaRegistered(
    const std::string& msg_type,
    converter::SchemaConverter& converter,
    const google::protobuf::DescriptorPool& descriptor_pool) {
  if (!converter.RegisterMessageByName(msg_type, &descriptor_pool)) {
    return false;
  }

  const auto schemas = converter.ExportAll();
  for (const auto& schema : schemas) {
    server_.RegisterSchema(schema);
  }
  return true;
}

}  // namespace bridge
}  // namespace autoviz



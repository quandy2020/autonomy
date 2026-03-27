#include "autonomy/autoviz/core/autolink/autolink_bridge.hpp"

#include <chrono>
#include <regex>
#include <vector>
#include <unistd.h>

#include "autonomy/autoviz/core/convert/message/convert_automsgs_message.hpp"
#include "autonomy/autoviz/core/foxglove/foxglove_server.hpp"
#include "autonomy/autoviz/core/recorder/recorder.hpp"
#include "autolink/autolink.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/service_discovery/specific_manager/channel_manager.hpp"
#include "autolink/service_discovery/topology_manager.hpp"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"

namespace autoviz {
namespace autolink {

AutolinkBridge::AutolinkBridge(const config::AutolinkConfig& cfg, server::FoxgloveServer& server,
                               recorder::Recorder* recorder)
    : cfg_(cfg), server_(server), recorder_(recorder) {}

AutolinkBridge::~AutolinkBridge() {
  Stop();
}

void AutolinkBridge::Start() {
  if (running_.exchange(true)) {
    return;
  }

  if (!node_) {
    node_ = autolink::CreateNode("autoviz_bridge_" + std::to_string(getpid()));
  }

  if (!node_) {
    running_.store(false);
    return;
  }

  discovery_running_.store(true);
  discovery_thread_ = std::thread(&AutolinkBridge::RunDiscoveryLoop, this);
}

void AutolinkBridge::Stop() {
  if (!running_.exchange(false)) {
    return;
  }

  discovery_running_.store(false);
  if (discovery_thread_.joinable()) {
    discovery_thread_.join();
  }

  readers_.clear();
  node_.reset();
}

void AutolinkBridge::RunDiscoveryLoop() {
  using namespace std::chrono_literals;

  while (discovery_running_.load()) {
    PollTopicsAndSubscribe();
    const auto period_ms = static_cast<int>(
        cfg_.min_update_period_ms > 0.0 ? cfg_.min_update_period_ms : 100.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}

bool AutolinkBridge::TopicMatchesWhitelist(const std::string& topic) const {
  if (cfg_.topic_whitelist.empty()) {
    return true;
  }
  for (const auto& pattern : cfg_.topic_whitelist) {
    try {
      std::regex re(pattern);
      if (std::regex_match(topic, re)) {
        return true;
      }
    } catch (const std::regex_error&) {
    }
  }
  return false;
}

void AutolinkBridge::PollTopicsAndSubscribe() {
  auto* topo = autolink::service_discovery::TopologyManager::Instance();
  if (topo == nullptr || topo->channel_manager() == nullptr) {
    return;
  }

  std::vector<std::string> channels;
  topo->channel_manager()->GetChannelNames(&channels);

  for (const auto& name : channels) {
    if (!TopicMatchesWhitelist(name)) {
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(subscribed_mutex_);
      if (subscribed_topics_.count(name) > 0) {
        continue;
      }
    }
    SubscribeToTopic(name);
  }
}

void AutolinkBridge::SubscribeToTopic(const std::string& topic_name) {
  auto* topo = autolink::service_discovery::TopologyManager::Instance();
  if (topo == nullptr || topo->channel_manager() == nullptr) {
    return;
  }

  const bool subscribed = SubscribeSerialized(
      topic_name,
      [this](const std::string& topic,
             const std::shared_ptr<autolink::message::RawMessage>& msg) {
        const auto ts = static_cast<std::uint64_t>(autolink::Time::Now().ToNanosecond());

        std::string msg_type;
        std::string proto_desc;
        auto* t = autolink::service_discovery::TopologyManager::Instance();
        if (t != nullptr && t->channel_manager() != nullptr) {
          t->channel_manager()->GetMsgType(topic, &msg_type);
          t->channel_manager()->GetProtoDesc(topic, &proto_desc);
        }

        const std::string schema_msg_type = msg_type.empty() ? topic : msg_type;

        const auto* descriptor_pool = google::protobuf::DescriptorPool::generated_pool();
        if (descriptor_pool != nullptr && !schema_msg_type.empty()) {
          (void)RegisterSchemaForType(schema_msg_type);
        }

        if (schema_msg_type.empty()) {
          return;
        }

        const std::string* proto_desc_ptr = msg_type.empty() ? nullptr : &proto_desc;
        PublishSerialized(topic, schema_msg_type, msg->message, ts, proto_desc_ptr);

        if (recorder_ != nullptr && recorder_->IsEnabled()) {
          (void)recorder_->AddTopic(topic, schema_msg_type, proto_desc);
          (void)recorder_->WriteSample(topic, msg->message.data(), msg->message.size(), ts, ts);
        }
      });

  if (subscribed) {
    std::lock_guard<std::mutex> lock(subscribed_mutex_);
    subscribed_topics_.insert(topic_name);
  }
}

void AutolinkBridge::PublishProtobuf(const std::string& topic, const std::string& msg_type,
                                     const google::protobuf::Message& msg,
                                     std::uint64_t timestamp_ns) {
  std::string buffer;
  if (!msg.SerializeToString(&buffer)) {
    return;
  }
  server_.EmitProtobuf(topic, msg_type, "protobuf", buffer.data(), buffer.size(), timestamp_ns,
                       /*proto_desc=*/nullptr);
}

void AutolinkBridge::PublishSerialized(const std::string& topic, const std::string& msg_type,
                                       const std::string& serialized_msg,
                                       std::uint64_t timestamp_ns,
                                       const std::string* proto_desc) {
  server_.EmitProtobuf(topic, msg_type, "protobuf", serialized_msg.data(), serialized_msg.size(),
                       timestamp_ns, proto_desc);

#if defined(AUTOVIZ_FOXGLOVE_HAS_ENCODE) && AUTOVIZ_FOXGLOVE_HAS_ENCODE
  converter::FoxgloveConvertedMessage cm;
  if (converter::ConvertMessageToFoxglove(msg_type, serialized_msg.data(), serialized_msg.size(),
                                          &cm) &&
      cm.ok && cm.schema_name != "foxglove.Log") {
    server_.EmitFoxglove(topic + "/foxglove", cm, timestamp_ns);
  }
#endif
}

bool AutolinkBridge::SubscribeSerialized(
    const std::string& topic,
    const std::function<void(const std::string&, const std::shared_ptr<autolink::message::RawMessage>&)>& cb) {
  return SubscribeProtobuf<autolink::message::RawMessage>(topic, cb);
}

bool AutolinkBridge::RegisterSchemaForType(const std::string& msg_type) {
  const auto* pool = google::protobuf::DescriptorPool::generated_pool();
  if (pool == nullptr) {
    return false;
  }
  if (!schema_converter_.AddFromFullName(msg_type, pool)) {
    return false;
  }

  for (const auto& schema : schema_converter_.ListSchemas()) {
    server_.RegisterSchema(schema);
  }
  return true;
}

}  // namespace autolink
}  // namespace autoviz

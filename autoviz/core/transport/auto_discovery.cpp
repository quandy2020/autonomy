#include "autonomy/autoviz/core/transport/auto_discovery.hpp"

#include <chrono>
#include <regex>

#include "autolink/autolink.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/service_discovery/specific_manager/channel_manager.hpp"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autonomy/autoviz/core/bridge/autolink_bridge.hpp"
#include "autonomy/autoviz/core/mcap/recorder.hpp"
#include "google/protobuf/descriptor.h"

namespace autoviz {
namespace transport {

AutoDiscovery::AutoDiscovery(const config::AutolinkConfig& cfg,
                             bridge::AutolinkBridge& bridge,
                             mcap::Recorder* recorder)
    : cfg_(cfg), bridge_(bridge), recorder_(recorder) {}

AutoDiscovery::~AutoDiscovery() {
  Stop();
}

void AutoDiscovery::Start() {
  if (running_.exchange(true)) {
    return;
  }

  discovery_thread_ = std::thread(&AutoDiscovery::DiscoveryLoop, this);
}

void AutoDiscovery::Stop() {
  if (!running_.exchange(false)) {
    return;
  }

  if (discovery_thread_.joinable()) {
    discovery_thread_.join();
  }
}

void AutoDiscovery::DiscoveryLoop() {
  using namespace std::chrono_literals;

  while (running_.load()) {
    DiscoverAndSubscribeOnce();

    const auto period_ms =
        static_cast<int>(cfg_.min_update_period_ms > 0.0 ? cfg_.min_update_period_ms : 100.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}

bool AutoDiscovery::MatchesWhitelist(const std::string& topic) const {
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
      // Ignore invalid regex and continue.
    }
  }
  return false;
}

void AutoDiscovery::DiscoverAndSubscribeOnce() {
  auto* topo = autolink::service_discovery::TopologyManager::Instance();
  if (topo == nullptr || topo->channel_manager() == nullptr) {
    return;
  }

  std::vector<std::string> channels;
  topo->channel_manager()->GetChannelNames(&channels);

  for (const auto& name : channels) {
    if (!MatchesWhitelist(name)) {
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(subscribed_mutex_);
      if (subscribed_topics_.count(name) > 0) {
        continue;
      }
    }

    SubscribeTopic(name);
  }
}

void AutoDiscovery::SubscribeTopic(const std::string& topic_name) {
  auto* topo = autolink::service_discovery::TopologyManager::Instance();
  if (topo == nullptr || topo->channel_manager() == nullptr) {
    return;
  }

  std::string msg_type;
  std::string proto_desc;
  topo->channel_manager()->GetMsgType(topic_name, &msg_type);
  topo->channel_manager()->GetProtoDesc(topic_name, &proto_desc);
  if (msg_type.empty()) {
    return;
  }

  const auto* descriptor_pool = google::protobuf::DescriptorPool::generated_pool();
  if (descriptor_pool == nullptr) {
    return;
  }
  if (!bridge_.EnsureSchemaRegistered(msg_type, schema_converter_, *descriptor_pool)) {
    return;
  }
  if (recorder_ != nullptr && recorder_->IsEnabled()) {
    recorder_->RegisterTopic(topic_name, msg_type, proto_desc);
  }

  const bool subscribed = bridge_.SubscribeRaw(
      topic_name,
      [this, msg_type](const std::string& topic,
                       const std::shared_ptr<autolink::message::RawMessage>& msg) {
        const auto ts = static_cast<std::uint64_t>(autolink::Time::Now().ToNanosecond());
        bridge_.ForwardRawToFoxglove(topic, msg_type, msg->message, ts);
        if (recorder_ != nullptr && recorder_->IsEnabled()) {
          recorder_->WriteMessage(topic, msg->message.data(), msg->message.size(), ts, ts);
        }
      });

  if (subscribed) {
    std::lock_guard<std::mutex> lock(subscribed_mutex_);
    subscribed_topics_.insert(topic_name);
  }
}

}  // namespace transport
}  // namespace autoviz



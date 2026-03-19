// Periodically discovers autolink channels and updates bridge subscriptions.
#pragma once

#include <atomic>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "autonomy/autoviz/core/config/config.hpp"
#include "autonomy/autoviz/core/converter/schema_converter.hpp"

namespace autoviz {

namespace bridge {
class AutolinkBridge;
}  // namespace bridge

namespace mcap {
class Recorder;
}  // namespace mcap

namespace transport {

class AutoDiscovery {
 public:
  AutoDiscovery(const config::AutolinkConfig& cfg,
                bridge::AutolinkBridge& bridge,
                mcap::Recorder* recorder);
  ~AutoDiscovery();

  void Start();
  void Stop();

 private:
  void DiscoveryLoop();
  bool MatchesWhitelist(const std::string& topic) const;
  void DiscoverAndSubscribeOnce();
  void SubscribeTopic(const std::string& topic_name);

 private:
  config::AutolinkConfig cfg_;
  bridge::AutolinkBridge& bridge_;
  mcap::Recorder* recorder_{nullptr};
  std::atomic<bool> running_{false};
  converter::SchemaConverter schema_converter_;

  std::thread discovery_thread_;
  std::set<std::string> subscribed_topics_;
  mutable std::mutex subscribed_mutex_;
};

}  // namespace transport
}  // namespace autoviz


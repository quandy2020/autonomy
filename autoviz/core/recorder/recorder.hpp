// MCAP recorder for offline foxglove visualization.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autonomy/autoviz/core/settings.hpp"

namespace autoviz {
namespace recorder {

class Recorder {
 public:
  explicit Recorder(const config::McapConfig& cfg);
  ~Recorder();

  bool Start();
  void Stop();
  bool IsEnabled() const;

  bool AddTopic(const std::string& topic, const std::string& msg_type,
                const std::string& proto_desc);

  bool WriteSample(const std::string& topic, const void* data, std::size_t size,
                   std::uint64_t log_time_ns, std::uint64_t publish_time_ns);

 private:
  struct TopicState {
    std::uint16_t channel_id{0};
    std::uint16_t schema_id{0};
    bool registered{false};
  };

  config::McapConfig cfg_;
  bool started_{false};
  std::mutex mutex_;
  std::uint16_t next_schema_id_{1};
  std::uint16_t next_channel_id_{1};
  std::unordered_map<std::string, TopicState> topic_states_;

#if defined(AUTOVIZ_HAS_MCAP)
  class Impl;
  std::unique_ptr<Impl> impl_;
#endif
};

}  // namespace recorder
}  // namespace autoviz


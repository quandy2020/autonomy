/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>

namespace autoviz {
namespace integration {

struct ChannelStats {
  std::uint64_t message_count = 0;
  double frequency_hz = 0.0;
};

/** Tracks live message counts and publish rates per Autolink channel. */
class ChannelStatsRegistry {
 public:
  static ChannelStatsRegistry& instance();

  void recordMessage(const std::string& channel);
  ChannelStats stats(const std::string& channel) const;
  void reset();

 private:
  ChannelStatsRegistry() = default;

  struct ChannelEntry {
    std::uint64_t message_count = 0;
    std::deque<std::chrono::steady_clock::time_point> recent_times;
  };

  void pruneOldSamples(ChannelEntry* entry,
                       std::chrono::steady_clock::time_point now) const;
  double computeFrequencyHz(const ChannelEntry& entry,
                          std::chrono::steady_clock::time_point now) const;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, ChannelEntry> channels_;
};

}  // namespace integration
}  // namespace autoviz

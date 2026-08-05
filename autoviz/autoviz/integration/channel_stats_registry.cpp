/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/channel_stats_registry.hpp"

namespace autoviz {
namespace integration {
namespace {

constexpr std::chrono::seconds kFrequencyWindow{1};

}  // namespace

ChannelStatsRegistry& ChannelStatsRegistry::instance() {
  static ChannelStatsRegistry registry;
  return registry;
}

void ChannelStatsRegistry::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  channels_.clear();
}

void ChannelStatsRegistry::pruneOldSamples(
    ChannelEntry* entry, std::chrono::steady_clock::time_point now) const {
  if (entry == nullptr) {
    return;
  }
  while (!entry->recent_times.empty() &&
         now - entry->recent_times.front() > kFrequencyWindow) {
    entry->recent_times.pop_front();
  }
}

double ChannelStatsRegistry::computeFrequencyHz(
    const ChannelEntry& entry, std::chrono::steady_clock::time_point now) const {
  if (entry.recent_times.empty()) {
    return 0.0;
  }
  const auto window_start = now - kFrequencyWindow;
  std::size_t count = 0;
  for (const auto& timestamp : entry.recent_times) {
    if (timestamp >= window_start) {
      ++count;
    }
  }
  return static_cast<double>(count);
}

void ChannelStatsRegistry::recordMessage(const std::string& channel) {
  if (channel.empty()) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  ChannelEntry& entry = channels_[channel];
  ++entry.message_count;
  entry.recent_times.push_back(now);
  pruneOldSamples(&entry, now);
  while (entry.recent_times.size() > 512) {
    entry.recent_times.pop_front();
  }
}

ChannelStats ChannelStatsRegistry::stats(const std::string& channel) const {
  ChannelStats result;
  if (channel.empty()) {
    return result;
  }
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = channels_.find(channel);
  if (found == channels_.end()) {
    return result;
  }
  ChannelEntry entry = found->second;
  pruneOldSamples(&entry, now);
  result.message_count = entry.message_count;
  result.frequency_hz = computeFrequencyHz(entry, now);
  return result;
}

}  // namespace integration
}  // namespace autoviz

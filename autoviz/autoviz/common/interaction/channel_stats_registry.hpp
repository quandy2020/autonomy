/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channel_stats_registry.hpp
 * @brief Live per-channel message counts and Hz estimates for Autoviz UI.
 *
 * Call @ref recordMessage() from reader / display paths; Channel Graph and
 * status panels query @ref stats().
 *
 * @see ChannelStats
 * @see ChannelReaderRegistry
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>

namespace autoviz {
namespace integration {

/**
 * @struct ChannelStats
 * @brief Snapshot of traffic for one Autolink channel.
 */
struct ChannelStats {
  /** Total messages recorded since last @ref ChannelStatsRegistry::reset(). */
  std::uint64_t message_count = 0;
  /** Sliding-window publish rate estimate in Hertz. */
  double frequency_hz = 0.0;
};

/**
 * @class ChannelStatsRegistry
 * @brief Process-wide singleton tracking message counts and rates per channel.
 *
 * Frequency is computed from a deque of recent arrival timestamps
 * (@c ChannelEntry::recent_times), pruned by @ref pruneOldSamples().
 *
 * @note Thread-safe.
 */
class ChannelStatsRegistry {
 public:
  /**
   * @brief Returns the process-wide registry singleton.
   * @return Reference to the shared instance.
   */
  static ChannelStatsRegistry& instance();

  /**
   * @brief Records one message arrival on @p channel (increments count / rate).
   * @param channel Fully-qualified Autolink channel name.
   */
  void recordMessage(const std::string& channel);

  /**
   * @brief Returns current stats for @p channel (zeros if never recorded).
   *
   * @param channel Fully-qualified Autolink channel name.
   * @return @ref ChannelStats snapshot.
   */
  ChannelStats stats(const std::string& channel) const;

  /**
   * @brief Clears all per-channel counters and rate windows.
   */
  void reset();

 private:
  /** @brief Private default constructor (singleton). */
  ChannelStatsRegistry() = default;

  /**
   * @struct ChannelEntry
   * @brief Internal counters and arrival timestamps for one channel.
   */
  struct ChannelEntry {
    /** Lifetime message count for this channel. */
    std::uint64_t message_count = 0;
    /** Recent arrival times for Hz estimation. */
    std::deque<std::chrono::steady_clock::time_point> recent_times;
  };

  /**
   * @brief Drops timestamps older than the rate window from @p entry.
   *
   * @param entry Channel entry to prune (non-null).
   * @param now Current steady-clock time.
   */
  void pruneOldSamples(ChannelEntry* entry,
                       std::chrono::steady_clock::time_point now) const;

  /**
   * @brief Computes Hz from remaining timestamps in @p entry.
   *
   * @param entry Channel entry after pruning.
   * @param now Current steady-clock time.
   * @return Frequency in Hertz, or @c 0.0 if insufficient samples.
   */
  double computeFrequencyHz(const ChannelEntry& entry,
                          std::chrono::steady_clock::time_point now) const;

  /** Guards @c channels_. */
  mutable std::mutex mutex_;

  /** Per-channel stats state. */
  std::unordered_map<std::string, ChannelEntry> channels_;
};

}  // namespace integration
}  // namespace autoviz

/*
 * Copyright 2026 The Openbot Authors
 *
 * Per-channel frequency / latency diagnostics (Phase 4).
 */

#pragma once

#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

#include "autonomy/orbisview/backend/common/stream_envelope.h"

namespace autonomy {
namespace orbisview {
namespace backend {

struct ChannelStats {
  uint64_t count{0};
  double hz{0.0};
  double latency_ms{0.0};
  int64_t last_timestamp_ns{0};
  std::chrono::steady_clock::time_point last_wall{};
};

class ChannelStatsTracker {
 public:
  void Observe(const core::StreamEnvelope& env) {
    using clock = std::chrono::steady_clock;
    const auto now = clock::now();
    const auto wall_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    std::lock_guard<std::mutex> lock(mutex_);
    auto& s = stats_[env.channel];
    if (s.count > 0) {
      const double dt =
          std::chrono::duration<double>(now - s.last_wall).count();
      if (dt > 1e-6) {
        const double inst = 1.0 / dt;
        s.hz = s.hz <= 0.0 ? inst : (0.8 * s.hz + 0.2 * inst);
      }
    }
    if (env.timestamp_ns > 0) {
      s.latency_ms =
          static_cast<double>(wall_ns - env.timestamp_ns) / 1.0e6;
    }
    s.last_timestamp_ns = env.timestamp_ns;
    s.last_wall = now;
    ++s.count;
  }

  std::string ToJson() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream oss;
    oss << "{\"op\":\"channel_stats\",\"channels\":[";
    bool first = true;
    for (const auto& kv : stats_) {
      if (!first) {
        oss << ',';
      }
      first = false;
      oss << "{\"name\":" << core::JsonEscape(kv.first)
          << ",\"count\":" << kv.second.count << ",\"hz\":" << kv.second.hz
          << ",\"latency_ms\":" << kv.second.latency_ms << '}';
    }
    oss << "]}";
    return oss.str();
  }

 private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, ChannelStats> stats_;
};

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy

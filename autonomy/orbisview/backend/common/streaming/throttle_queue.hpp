/*
 * Copyright 2026 The Openbot Authors
 *
 * Latest-frame-wins bounded queue. Safe to Push from Autolink callbacks;
 * Pop from the streaming / WebSocket thread only.
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "autonomy/orbisview/backend/common/stream_envelope.hpp"

namespace autonomy {
namespace orbisview {
namespace backend {

class ThrottleQueue {
 public:
  explicit ThrottleQueue(size_t per_channel_capacity = 1)
      : capacity_(per_channel_capacity == 0 ? 1 : per_channel_capacity) {}

  /** Push latest envelope for channel; drops older pending frame. */
  void Push(core::StreamEnvelope env) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& slot = slots_[env.channel];
    if (slot.has_value()) {
      ++dropped_;
    }
    slot = std::move(env);
  }

  /** Pop one pending channel (round-robin-ish via unordered iteration). */
  std::optional<core::StreamEnvelope> Pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = slots_.begin(); it != slots_.end(); ++it) {
      if (it->second.has_value()) {
        auto out = std::move(*it->second);
        it->second.reset();
        return out;
      }
    }
    return std::nullopt;
  }

  uint64_t DroppedFrames() const { return dropped_.load(); }

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    slots_.clear();
  }

 private:
  size_t capacity_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::optional<core::StreamEnvelope>> slots_;
  std::atomic<uint64_t> dropped_{0};
};

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy

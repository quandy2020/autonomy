/*
 * Copyright 2026 The Openbot Authors
 *
 * Latest-frame-wins bounded queue. Safe to Push from Autolink callbacks;
 * Pop / WaitPop from the streaming / WebSocket thread only.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

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
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto& slot = slots_[env.channel];
      if (slot.has_value()) {
        ++dropped_;
      }
      slot = std::move(env);
    }
    cv_.notify_one();
  }

  /** Pop one pending channel (unordered). */
  std::optional<core::StreamEnvelope> Pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    return PopUnlocked();
  }

  /**
   * Wait until a frame is available or timeout elapses.
   * Returns nullopt on timeout (caller should check shutdown).
   */
  template <typename Rep, typename Period>
  std::optional<core::StreamEnvelope> WaitPopFor(
      const std::chrono::duration<Rep, Period>& timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [this] {
          return stopping_ || HasPendingUnlocked();
        })) {
      return std::nullopt;
    }
    if (stopping_) return std::nullopt;
    return PopUnlocked();
  }

  /** Drain every pending channel once (latest frame each). */
  std::vector<core::StreamEnvelope> PopAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<core::StreamEnvelope> out;
    out.reserve(slots_.size());
    for (auto& kv : slots_) {
      if (kv.second.has_value()) {
        out.push_back(std::move(*kv.second));
        kv.second.reset();
      }
    }
    return out;
  }

  /** Wake waiters (call from Stop before joining the pump thread). */
  void Wake() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    cv_.notify_all();
  }

  uint64_t DroppedFrames() const { return dropped_.load(); }

  void Clear() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slots_.clear();
      stopping_ = false;
    }
    cv_.notify_all();
  }

 private:
  bool HasPendingUnlocked() const {
    for (const auto& kv : slots_) {
      if (kv.second.has_value()) return true;
    }
    return false;
  }

  std::optional<core::StreamEnvelope> PopUnlocked() {
    for (auto it = slots_.begin(); it != slots_.end(); ++it) {
      if (it->second.has_value()) {
        auto out = std::move(*it->second);
        it->second.reset();
        return out;
      }
    }
    return std::nullopt;
  }

  size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool stopping_{false};
  std::unordered_map<std::string, std::optional<core::StreamEnvelope>> slots_;
  std::atomic<uint64_t> dropped_{0};
};

}  // namespace backend
}  // namespace orbisview
}  // namespace autonomy

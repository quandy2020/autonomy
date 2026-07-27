/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <string>

namespace autoviz {
namespace integration {

/** Thread-safe bounded queue for autolink callback → UI thread handoff. */
class MessageQueue {
 public:
  void push(std::string payload) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= kMaxSize) {
      queue_.pop_front();
    }
    queue_.push_back(std::move(payload));
  }

  std::optional<std::string> pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }
    std::string payload = std::move(queue_.front());
    queue_.pop_front();
    return payload;
  }

 private:
  static constexpr std::size_t kMaxSize = 256;
  std::mutex mutex_;
  std::deque<std::string> queue_;
};

}  // namespace integration
}  // namespace autoviz

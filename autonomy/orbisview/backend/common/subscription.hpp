/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <chrono>
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace autonomy {
namespace orbisview {
namespace core {

struct SubscriptionOptions {
  double max_hz{0.0};  // 0 = unlimited
  size_t queue_capacity{1};
};

struct SubscriptionState {
  SubscriptionOptions options;
  uint64_t dropped{0};
  uint64_t delivered{0};
  std::chrono::steady_clock::time_point last_send{};
};

}  // namespace core
}  // namespace orbisview
}  // namespace autonomy

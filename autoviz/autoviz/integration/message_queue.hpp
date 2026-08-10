/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace autoviz {
namespace integration {

/** Thread-safe latest-only queue for autolink callback → UI thread handoff.
 *  Never drains a backlog for rendering — only the newest sample is kept. */
class MessageQueue {
 public:
  MessageQueue();
  ~MessageQueue();

  MessageQueue(const MessageQueue&) = delete;
  MessageQueue& operator=(const MessageQueue&) = delete;
  MessageQueue(MessageQueue&& other) noexcept;
  MessageQueue& operator=(MessageQueue&& other) noexcept;

  /** When false, push() drops payloads (app background / paused). */
  static void setAcceptIncoming(bool accept);
  static bool acceptIncoming();

  /** Clear every live MessageQueue (drop background backlog before resume). */
  static void clearAllPending();

  void push(std::string payload);

  std::optional<std::string> pop();

  /** Drop any older samples and return only the newest payload. */
  std::optional<std::string> takeLatest();

  void clear();

 private:
  static std::mutex& registryMutex();
  static std::vector<MessageQueue*>& registry();
  void registerSelf();
  void unregisterSelf();

  static std::atomic<bool> accept_incoming_;

  std::mutex mutex_;
  std::deque<std::string> queue_;
};

}  // namespace integration
}  // namespace autoviz

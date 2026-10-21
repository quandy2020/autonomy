/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/interaction/message_queue.hpp"

#include <algorithm>
#include <utility>

namespace autoviz {
namespace integration {

std::atomic<bool> MessageQueue::accept_incoming_{true};

std::mutex& MessageQueue::registryMutex() {
  static std::mutex mutex;
  return mutex;
}

std::vector<MessageQueue*>& MessageQueue::registry() {
  static std::vector<MessageQueue*> queues;
  return queues;
}

void MessageQueue::registerSelf() {
  std::lock_guard<std::mutex> lock(registryMutex());
  registry().push_back(this);
}

void MessageQueue::unregisterSelf() {
  std::lock_guard<std::mutex> lock(registryMutex());
  auto& queues = registry();
  queues.erase(std::remove(queues.begin(), queues.end(), this), queues.end());
}

MessageQueue::MessageQueue() { registerSelf(); }

MessageQueue::~MessageQueue() { unregisterSelf(); }

MessageQueue::MessageQueue(MessageQueue&& other) noexcept {
  {
    std::lock_guard<std::mutex> lock(other.mutex_);
    queue_ = std::move(other.queue_);
    max_depth_ = other.max_depth_;
  }
  // Point registry entry at the moved-to object.
  std::lock_guard<std::mutex> lock(registryMutex());
  auto& queues = registry();
  for (MessageQueue*& entry : queues) {
    if (entry == &other) {
      entry = this;
      return;
    }
  }
  queues.push_back(this);
}

MessageQueue& MessageQueue::operator=(MessageQueue&& other) noexcept {
  if (this == &other) {
    return *this;
  }
  {
    std::scoped_lock lock(mutex_, other.mutex_);
    queue_ = std::move(other.queue_);
    max_depth_ = other.max_depth_;
  }
  return *this;
}

void MessageQueue::setAcceptIncoming(bool accept) {
  accept_incoming_.store(accept, std::memory_order_relaxed);
}

bool MessageQueue::acceptIncoming() {
  return accept_incoming_.load(std::memory_order_relaxed);
}

void MessageQueue::clearAllPending() {
  std::vector<MessageQueue*> queues;
  {
    std::lock_guard<std::mutex> lock(registryMutex());
    queues = registry();
  }
  for (MessageQueue* queue : queues) {
    if (queue != nullptr) {
      queue->clear();
    }
  }
}

void MessageQueue::setMaxDepth(std::size_t depth) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_depth_ = std::max<std::size_t>(1, depth);
  while (queue_.size() > max_depth_) {
    queue_.pop_front();
  }
}

std::size_t MessageQueue::maxDepth() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return max_depth_;
}

void MessageQueue::push(std::string payload) {
  if (!accept_incoming_.load(std::memory_order_relaxed)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (max_depth_ <= 1) {
    // Latest-only: never retain a backlog for the UI thread to catch up.
    queue_.clear();
    queue_.push_back(std::move(payload));
    return;
  }
  queue_.push_back(std::move(payload));
  while (queue_.size() > max_depth_) {
    queue_.pop_front();
  }
}

std::optional<std::string> MessageQueue::pop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty()) {
    return std::nullopt;
  }
  std::string payload = std::move(queue_.front());
  queue_.pop_front();
  return payload;
}

std::optional<std::string> MessageQueue::takeLatest() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty()) {
    return std::nullopt;
  }
  std::string payload = std::move(queue_.back());
  queue_.clear();
  return payload;
}

void MessageQueue::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.clear();
}

}  // namespace integration
}  // namespace autoviz

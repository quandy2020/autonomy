/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file message_queue.hpp
 * @brief Thread-safe latest-only queue for Autolink callback → Qt UI handoff.
 *
 * Reader callbacks push payloads; the UI thread pops or @ref takeLatest().
 * Never drains a backlog for rendering — only the newest sample is kept when
 * using @ref takeLatest(), matching RViz “latest message” display semantics.
 *
 * @see ChannelReaderRegistry
 * @see setAcceptIncoming
 */

#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace autoviz {
namespace integration {

/**
 * @class MessageQueue
 * @brief Per-display (or per-consumer) queue with process-wide pause / clear.
 *
 * ## Global controls
 *
 * - @ref setAcceptIncoming(@c false): @ref push() drops payloads (app
 *   background / paused playback).
 * - @ref clearAllPending(): drops every live queue before resume.
 *
 * Move-only; copy is deleted. Instances self-register for @ref clearAllPending().
 */
class MessageQueue {
 public:
  /** @brief Constructs an empty queue and registers it process-wide. */
  MessageQueue();

  /** @brief Unregisters and destroys the queue. */
  ~MessageQueue();

  MessageQueue(const MessageQueue&) = delete;
  MessageQueue& operator=(const MessageQueue&) = delete;

  /**
   * @brief Move-constructs, transferring pending payloads and registry slot.
   * @param other Source queue (left empty / unregistered).
   */
  MessageQueue(MessageQueue&& other) noexcept;

  /**
   * @brief Move-assigns, transferring pending payloads and registry slot.
   * @param other Source queue (left empty / unregistered).
   * @return @c *this.
   */
  MessageQueue& operator=(MessageQueue&& other) noexcept;

  /**
   * @brief Process-wide gate: when @c false, all @ref push() calls drop data.
   *
   * Used when the app is backgrounded or playback is paused so queues do not
   * accumulate stale samples.
   *
   * @param accept Whether new payloads should be accepted.
   * @see acceptIncoming()
   */
  static void setAcceptIncoming(bool accept);

  /**
   * @brief Returns whether @ref push() currently accepts payloads.
   * @return Process-wide accept flag.
   */
  static bool acceptIncoming();

  /**
   * @brief Clears every live @ref MessageQueue (drop backlog before resume).
   */
  static void clearAllPending();

  /**
   * @brief Enqueues @p payload if incoming is accepted.
   *
   * @param payload Opaque channel payload string (moved in).
   */
  void push(std::string payload);

  /**
   * @brief Pops the oldest queued payload (FIFO), if any.
   * @return Payload, or @c std::nullopt if empty.
   */
  std::optional<std::string> pop();

  /**
   * @brief Drops older samples and returns only the newest payload.
   *
   * Preferred path for displays that should not process a backlog.
   *
   * @return Newest payload, or @c std::nullopt if empty.
   */
  std::optional<std::string> takeLatest();

  /**
   * @brief Removes all pending payloads from this queue.
   */
  void clear();

 private:
  /** @brief Mutex protecting the process-wide registry of live queues. */
  static std::mutex& registryMutex();

  /** @brief Mutable list of live @ref MessageQueue instances. */
  static std::vector<MessageQueue*>& registry();

  /** @brief Adds @c this to the process-wide registry. */
  void registerSelf();

  /** @brief Removes @c this from the process-wide registry. */
  void unregisterSelf();

  /** Process-wide accept gate for @ref push(). */
  static std::atomic<bool> accept_incoming_;

  /** Guards @c queue_. */
  std::mutex mutex_;

  /** Pending payloads (FIFO; @ref takeLatest() discards all but the last). */
  std::deque<std::string> queue_;
};

}  // namespace integration
}  // namespace autoviz

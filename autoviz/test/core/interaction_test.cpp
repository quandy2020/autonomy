/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file interaction_test.cpp
 * @brief Core (non-Qt) tests for the Autolink callback -> UI thread queue.
 */

#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "autoviz/common/interaction/message_queue.hpp"

namespace {

using autoviz::integration::MessageQueue;

int g_failures = 0;

void Check(bool condition, const char* what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_failures;
  }
}

void TestLatestOnly() {
  MessageQueue queue;
  queue.push("first");
  queue.push("second");
  const auto latest = queue.takeLatest();
  Check(latest.has_value() && *latest == "second",
        "latest-only queue returns the newest payload");
  Check(!queue.takeLatest().has_value(),
        "latest-only queue is empty after takeLatest");
}

void TestDepthFifo() {
  MessageQueue queue;
  queue.setMaxDepth(3);
  queue.push("a");
  queue.push("b");
  queue.push("c");
  queue.push("d");
  Check(queue.pop().value_or("") == "b",
        "depth>1 drops the oldest payload (b first)");
  Check(queue.pop().value_or("") == "c", "FIFO keeps c");
  Check(queue.pop().value_or("") == "d", "FIFO keeps d");
  Check(!queue.pop().has_value(), "FIFO drains after the cap");
}

void TestClearAllReachesMoveConstructedQueue() {
  auto source = std::make_unique<MessageQueue>();
  source->push("payload");
  MessageQueue moved(std::move(*source));
  source.reset();  // moved-from queue dies; must not unregister `moved`
  MessageQueue::clearAllPending();
  Check(!moved.takeLatest().has_value(),
        "clearAllPending clears a move-constructed queue");
}

void TestMoveAssignReRegistersTarget() {
  MessageQueue moved_from;
  MessageQueue source;
  MessageQueue sink(std::move(moved_from));  // sink owns the registry slot
  // moved_from has no registry slot any more; assigning into it must put it
  // back so clearAllPending() can still reach the queue.
  moved_from = std::move(source);
  moved_from.push("pending");
  MessageQueue::clearAllPending();
  Check(!moved_from.takeLatest().has_value(),
        "clearAllPending clears a queue used as move-assign target");
}

}  // namespace

int main() {
  TestLatestOnly();
  TestDepthFifo();
  TestClearAllReachesMoveConstructedQueue();
  TestMoveAssignReRegistersTarget();
  if (g_failures != 0) {
    std::fprintf(stderr, "%d interaction test(s) failed\n", g_failures);
    return 1;
  }
  std::printf("interaction tests passed\n");
  return 0;
}


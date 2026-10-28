#include "common/thread_safe_queue.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace argus {
namespace common {
namespace {

TEST(ThreadSafeQueue, RejectsZeroCapacity) {
  EXPECT_THROW(ThreadSafeQueue<int>(0), std::invalid_argument);
}

TEST(ThreadSafeQueue, TryPushFailsWhenFull) {
  ThreadSafeQueue<int> queue(1);
  EXPECT_TRUE(queue.TryPush(1));
  EXPECT_FALSE(queue.TryPush(2));
  int value = 0;
  ASSERT_TRUE(queue.TryPop(&value));
  EXPECT_EQ(value, 1);
  EXPECT_TRUE(queue.TryPush(3));
  ASSERT_TRUE(queue.TryPop(&value));
  EXPECT_EQ(value, 3);
}

TEST(ThreadSafeQueue, CloseUnblocksWaitAndKeepsQueuedItems) {
  ThreadSafeQueue<int> queue(4);
  ASSERT_TRUE(queue.TryPush(5));
  ASSERT_TRUE(queue.TryPush(6));
  queue.Close();
  EXPECT_FALSE(queue.TryPush(7));

  int value = 0;
  ASSERT_TRUE(queue.WaitPop(&value));
  EXPECT_EQ(value, 5);
  ASSERT_TRUE(queue.TryPop(&value));
  EXPECT_EQ(value, 6);
  EXPECT_FALSE(queue.WaitPop(&value));
}

TEST(ThreadSafeQueue, WaitPopWakesOnClose) {
  ThreadSafeQueue<int> queue(1);
  std::atomic<bool> started{false};
  std::atomic<bool> returned_false{false};
  std::thread waiter([&] {
    started.store(true);
    int value = 0;
    returned_false.store(!queue.WaitPop(&value));
  });
  while (!started.load()) {
    std::this_thread::yield();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  queue.Close();
  waiter.join();
  EXPECT_TRUE(returned_false.load());
}

TEST(ThreadSafeQueue, NullOutputIsRejected) {
  ThreadSafeQueue<int> queue(1);
  EXPECT_FALSE(queue.TryPop(nullptr));
  EXPECT_FALSE(queue.WaitPop(nullptr));
}

}  // namespace
}  // namespace common
}  // namespace argus

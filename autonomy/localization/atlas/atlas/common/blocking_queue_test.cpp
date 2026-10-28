#include "atlas/common/blocking_queue.hpp"

#include <future>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas {
namespace {

TEST(BlockingQueueTest, CloseUnblocksConsumer) {
    BlockingQueue<int> queue(1);
    auto consumer = std::async(std::launch::async, [&queue] {
        int value = 0;
        return queue.Pop(&value);
    });
    queue.Close();
    EXPECT_FALSE(consumer.get());
}

TEST(BlockingQueueTest, PreservesOrder) {
    BlockingQueue<int> queue;
    ASSERT_TRUE(queue.Push(1));
    ASSERT_TRUE(queue.Push(2));
    int value = 0;
    ASSERT_TRUE(queue.Pop(&value));
    EXPECT_EQ(value, 1);
    ASSERT_TRUE(queue.Pop(&value));
    EXPECT_EQ(value, 2);
}

}  // namespace
}  // namespace autonomy::localization::atlas

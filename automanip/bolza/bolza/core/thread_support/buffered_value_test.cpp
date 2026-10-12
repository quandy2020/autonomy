#include <gtest/gtest.h>
#include <bolza/core/thread_support/buffered_value.hpp>

TEST(testBufferedValue, basicSetGet) {
  // initialize
  const std::string initialValue{"init"};
  bolza::BufferedValue<std::string> bufferedValue(initialValue);
  ASSERT_EQ(bufferedValue.get(), initialValue);

  // set buffer with copy
  const std::string updatedValue{"update"};
  bufferedValue.SetBuffer(updatedValue);
  ASSERT_EQ(bufferedValue.get(), initialValue);

  // update
  const bool isUpdated = bufferedValue.UpdateFromBuffer();
  ASSERT_TRUE(isUpdated);
  ASSERT_EQ(bufferedValue.get(), updatedValue);

  // update twice is false
  const bool isUpdatedTwice = bufferedValue.UpdateFromBuffer();
  ASSERT_FALSE(isUpdatedTwice);

  // set buffer with r-value
  bufferedValue.SetBuffer("update again");
  ASSERT_EQ(bufferedValue.get(), updatedValue);

  // update again
  const bool isUpdatedAgain = bufferedValue.UpdateFromBuffer();
  ASSERT_TRUE(isUpdatedAgain);
  ASSERT_EQ(bufferedValue.get(), "update again");
}

namespace {
/**
 * Move only class that counts the amount of times it has been moved.
 */
class MoveCounter {
 public:
  MoveCounter() : count_(0){};
  MoveCounter(const MoveCounter&) = delete;
  MoveCounter& operator=(const MoveCounter&) = delete;
  MoveCounter(MoveCounter&& other) : count_(other.count_ + 1){};
  MoveCounter& operator=(MoveCounter&& other) {
    count_ = other.count_ + 1;
    return *this;
  }

  int getCount() const { return count_; }

 private:
  int count_;
};
}  // unnamed namespace

TEST(testBufferedValue, moveCount) {
  // 1 move on creation
  bolza::BufferedValue<MoveCounter> bufferedValue(MoveCounter{});
  ASSERT_EQ(bufferedValue.get().getCount(), 1);

  /*
   * A new value can be set with two moves:
   *  - 1 into the buffer.
   *  - 1 from the buffer to the active value.
   */
  MoveCounter newCounter{};
  bufferedValue.SetBuffer(std::move(newCounter));
  const bool isUpdated = bufferedValue.UpdateFromBuffer();
  ASSERT_TRUE(isUpdated);
  ASSERT_EQ(bufferedValue.get().getCount(), 2);
}
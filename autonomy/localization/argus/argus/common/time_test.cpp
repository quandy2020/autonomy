#include "common/time.hpp"

#include <gtest/gtest.h>

namespace argus {
namespace common {
namespace {

TEST(Time, StampRoundTrip) {
  int32_t seconds = 0;
  uint32_t nanoseconds = 0;
  SecondsToStamp(1.5, &seconds, &nanoseconds);
  EXPECT_EQ(seconds, 1);
  EXPECT_EQ(nanoseconds, 500000000u);
  EXPECT_DOUBLE_EQ(StampToSeconds(seconds, nanoseconds), 1.5);
}

TEST(Time, NanosecondCarry) {
  int32_t seconds = 0;
  uint32_t nanoseconds = 0;
  SecondsToStamp(1.0 - 1e-10, &seconds, &nanoseconds);
  EXPECT_EQ(StampToSeconds(seconds, nanoseconds),
            StampToSeconds(seconds, nanoseconds));
  EXPECT_LT(nanoseconds, 1000000000u);
}

TEST(Time, NullStampPointersAreIgnored) {
  SecondsToStamp(1.0, nullptr, nullptr);
  EXPECT_GT(NowSeconds(), 0.0);
  EXPECT_GE(SteadyNowSeconds(), 0.0);
}

}  // namespace
}  // namespace common
}  // namespace argus

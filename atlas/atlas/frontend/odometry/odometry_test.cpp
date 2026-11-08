/*
 * Copyright 2026 The Openbot Authors
 */

#include "frontend/odometry/odometry.hpp"

#include <gtest/gtest.h>

namespace atlas {
namespace frontend {
namespace odometry {
namespace {

TEST(Odometry, DegradationCountUsesHysteresis) {
  EXPECT_EQ(UpdateDegradationCount(3, true), 4);
  EXPECT_EQ(UpdateDegradationCount(3, false), 2);
  EXPECT_EQ(UpdateDegradationCount(0, false), 0);
}

}  // namespace
}  // namespace odometry
}  // namespace frontend
}  // namespace atlas

#include "common/logging.hpp"

#include <gtest/gtest.h>

namespace atlas {
namespace common {
namespace {

TEST(Logging, InitAndShutdownAreIdempotent) {
  EXPECT_FALSE(IsLoggingInitialized());
  InitLogging("atlas_test");
  EXPECT_TRUE(IsLoggingInitialized());
  InitLogging("atlas_test");
  EXPECT_TRUE(IsLoggingInitialized());
  SetMinLogSeverity(google::GLOG_ERROR);
  EXPECT_EQ(FLAGS_minloglevel, google::GLOG_ERROR);
  ShutdownLogging();
  EXPECT_FALSE(IsLoggingInitialized());
  ShutdownLogging();
}

}  // namespace
}  // namespace common
}  // namespace atlas

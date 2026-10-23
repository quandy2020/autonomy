#include "common/thread_pool.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <stdexcept>
#include <vector>

namespace atlas::common {
namespace {

TEST(ThreadPool, PropagatesTaskFailureAndKeepsWorkerAvailable) {
  ThreadPool pool(1);
  auto failure = pool.Submit([] { throw std::runtime_error("task failure"); });
  EXPECT_THROW(failure.get(), std::runtime_error);
  EXPECT_EQ(pool.Submit([] { return 42; }).get(), 42);
}

TEST(ThreadPool, ShutdownDrainsAcceptedTasksAndRejectsNewWork) {
  ThreadPool pool(3);
  std::atomic<int> completed_count{0};
  std::vector<std::future<void>> task_results;
  for (int task_index = 0; task_index < 128; ++task_index) {
    task_results.push_back(pool.Submit([&] { ++completed_count; }));
  }
  pool.Shutdown();
  EXPECT_EQ(completed_count.load(), 128);
  for (auto& task_result : task_results) {
    EXPECT_NO_THROW(task_result.get());
  }
  EXPECT_THROW(pool.Submit([] {}), std::logic_error);
  EXPECT_NO_THROW(pool.Shutdown());
}

TEST(ThreadPool, SupportsMoveOnlyWorkThroughSchedulingInterface) {
  ThreadPool pool(1);
  ThreadPoolInterface& scheduler = pool;
  auto task_result =
      scheduler.Submit([value = std::make_unique<int>(7)] { return *value; });
  EXPECT_EQ(task_result.get(), 7);
  EXPECT_THROW(scheduler.Schedule({}), std::invalid_argument);
}

}  // namespace
}  // namespace atlas::common

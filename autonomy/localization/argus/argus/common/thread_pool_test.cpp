#include "common/thread_pool.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "common/task.hpp"

namespace argus::common {
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
  EXPECT_THROW(scheduler.Schedule(std::function<void()>()),
               std::invalid_argument);
}

TEST(ThreadPool, ExecutesDependencyGraphInOrder) {
  ThreadPool pool(3);
  std::mutex order_mutex;
  std::vector<int> execution_order;

  auto first = std::make_unique<Task>();
  first->SetWorkItem([&] {
    std::lock_guard<std::mutex> lock(order_mutex);
    execution_order.push_back(1);
  });
  const std::weak_ptr<Task> first_handle = pool.Schedule(std::move(first));

  auto second = std::make_unique<Task>();
  second->AddDependency(first_handle);
  second->SetWorkItem([&] {
    std::lock_guard<std::mutex> lock(order_mutex);
    execution_order.push_back(2);
  });
  const std::weak_ptr<Task> second_handle = pool.Schedule(std::move(second));

  std::promise<void> graph_completed;
  auto third = std::make_unique<Task>();
  third->AddDependency(second_handle);
  third->SetWorkItem([&] {
    {
      std::lock_guard<std::mutex> lock(order_mutex);
      execution_order.push_back(3);
    }
    graph_completed.set_value();
  });
  pool.Schedule(std::move(third));

  graph_completed.get_future().wait();
  EXPECT_EQ(execution_order, (std::vector<int>{1, 2, 3}));
}

TEST(ThreadPool, WaitsForAllDependenciesBeforeDispatch) {
  ThreadPool pool(2);
  std::promise<void> release_first;
  std::promise<void> first_started;

  auto first = std::make_unique<Task>();
  first->SetWorkItem([&] {
    first_started.set_value();
    release_first.get_future().wait();
  });
  const std::weak_ptr<Task> first_handle = pool.Schedule(std::move(first));
  first_started.get_future().wait();

  std::promise<void> dependent_completed;
  auto dependent = std::make_unique<Task>();
  dependent->AddDependency(first_handle);
  dependent->SetWorkItem([&] { dependent_completed.set_value(); });
  const std::weak_ptr<Task> dependent_handle =
      pool.Schedule(std::move(dependent));

  const std::shared_ptr<Task> dependent_task = dependent_handle.lock();
  ASSERT_NE(dependent_task, nullptr);
  EXPECT_EQ(dependent_task->state(), Task::State::kDispatched);
  release_first.set_value();
  dependent_completed.get_future().wait();
}

TEST(ThreadPool, FailedDependencyStillReleasesDependentTask) {
  ThreadPool pool(1);
  std::promise<void> release_failure;
  std::promise<void> failing_task_started;
  auto failing_task = std::make_unique<Task>();
  failing_task->SetWorkItem([&] {
    failing_task_started.set_value();
    release_failure.get_future().wait();
    throw std::runtime_error("failure");
  });
  const std::weak_ptr<Task> failing_handle =
      pool.Schedule(std::move(failing_task));
  failing_task_started.get_future().wait();

  std::promise<void> dependent_completed;
  auto dependent = std::make_unique<Task>();
  dependent->AddDependency(failing_handle);
  dependent->SetWorkItem([&] { dependent_completed.set_value(); });
  pool.Schedule(std::move(dependent));

  release_failure.set_value();
  dependent_completed.get_future().wait();
  EXPECT_EQ(pool.Submit([] { return 9; }).get(), 9);
}

}  // namespace
}  // namespace argus::common

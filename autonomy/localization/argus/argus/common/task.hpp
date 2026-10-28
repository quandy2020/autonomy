/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

/**
 * @file task.hpp
 * @brief Dependency-aware asynchronous task state machine.
 */

#ifndef ARGUS_COMMON_TASK_HPP_
#define ARGUS_COMMON_TASK_HPP_

#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <set>

namespace argus {
namespace common {

class ThreadPoolInterface;

/**
 * @brief A unit of work that becomes runnable after all dependencies finish.
 *
 * A task is configured while new, then scheduled exactly once. Dependencies
 * may complete before or after scheduling. A failed dependency still counts
 * as completed; callers can inspect each dependency's state and failure.
 */
class Task {
 public:
  using WorkItem = std::function<void()>;

  enum class State {
    kNew,
    kDispatched,
    kDependenciesCompleted,
    kRunning,
    kCompleted,
    kFailed,
  };

  Task() = default;
  ~Task() = default;

  Task(const Task&) = delete;
  Task& operator=(const Task&) = delete;

  State state() const;
  std::exception_ptr failure() const;

  /** @throws std::logic_error unless the task is new. */
  void SetWorkItem(WorkItem work_item);

  /**
   * @brief Add a dependency while this task is new.
   *
   * An expired weak pointer represents an already completed dependency.
   * @throws std::logic_error unless this task is new.
   * @throws std::invalid_argument for a self-dependency or duplicate.
   */
  void AddDependency(const std::weak_ptr<Task>& dependency);

 private:
  friend class ThreadPoolInterface;

  void AddDependentTask(Task* dependent_task);
  void Execute();
  void SetThreadPool(ThreadPoolInterface* thread_pool);
  void OnDependencyCompleted();

  mutable std::mutex mutex_;
  WorkItem work_item_;
  ThreadPoolInterface* thread_pool_ = nullptr;
  State state_ = State::kNew;
  std::size_t incomplete_dependency_count_ = 0;
  std::set<Task*> dependent_tasks_;
  std::exception_ptr failure_;
};

}  // namespace common
}  // namespace argus

#endif  // ARGUS_COMMON_TASK_HPP_

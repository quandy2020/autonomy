/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "common/task.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

#include "common/thread_pool.hpp"

namespace argus {
namespace common {

Task::State Task::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::exception_ptr Task::failure() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return failure_;
}

void Task::SetWorkItem(WorkItem work_item) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::kNew) {
    throw std::logic_error("work item can only be set on a new task");
  }
  work_item_ = std::move(work_item);
}

void Task::AddDependency(const std::weak_ptr<Task>& dependency) {
  std::shared_ptr<Task> shared_dependency = dependency.lock();
  if (shared_dependency && shared_dependency.get() == this) {
    throw std::invalid_argument("a task cannot depend on itself");
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::kNew) {
      throw std::logic_error("dependencies can only be added to a new task");
    }
    if (!shared_dependency) {
      return;
    }
    ++incomplete_dependency_count_;
  }
  try {
    shared_dependency->AddDependentTask(this);
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    --incomplete_dependency_count_;
    throw;
  }
}

void Task::AddDependentTask(Task* dependent_task) {
  bool dependency_already_finished = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dependency_already_finished =
        state_ == State::kCompleted || state_ == State::kFailed;
    if (!dependency_already_finished &&
        !dependent_tasks_.insert(dependent_task).second) {
      throw std::invalid_argument("dependency was added more than once");
    }
  }
  if (dependency_already_finished) {
    dependent_task->OnDependencyCompleted();
  }
}

void Task::SetThreadPool(ThreadPoolInterface* thread_pool) {
  bool dependencies_completed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::kNew) {
      throw std::logic_error("a task can only be scheduled once");
    }
    if (thread_pool == nullptr) {
      throw std::invalid_argument("thread pool must not be null");
    }
    thread_pool_ = thread_pool;
    state_ = State::kDispatched;
    dependencies_completed = incomplete_dependency_count_ == 0;
    if (dependencies_completed) {
      state_ = State::kDependenciesCompleted;
    }
  }
  if (dependencies_completed) {
    thread_pool->NotifyDependenciesCompleted(this);
  }
}

void Task::OnDependencyCompleted() {
  ThreadPoolInterface* thread_pool = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::kNew && state_ != State::kDispatched) {
      throw std::logic_error("dependency completed in an invalid task state");
    }
    if (incomplete_dependency_count_ == 0) {
      throw std::logic_error("dependency completion count underflow");
    }
    --incomplete_dependency_count_;
    if (incomplete_dependency_count_ == 0 && state_ == State::kDispatched) {
      state_ = State::kDependenciesCompleted;
      thread_pool = thread_pool_;
    }
  }
  if (thread_pool != nullptr) {
    thread_pool->NotifyDependenciesCompleted(this);
  }
}

void Task::Execute() {
  WorkItem work_item;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::kDependenciesCompleted) {
      throw std::logic_error("task executed before dependencies completed");
    }
    state_ = State::kRunning;
    work_item = work_item_;
  }

  std::exception_ptr failure;
  try {
    if (work_item) {
      work_item();
    }
  } catch (...) {
    failure = std::current_exception();
  }

  std::vector<Task*> dependent_tasks;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    failure_ = failure;
    state_ = failure ? State::kFailed : State::kCompleted;
    dependent_tasks.assign(dependent_tasks_.begin(), dependent_tasks_.end());
    dependent_tasks_.clear();
  }
  for (Task* dependent_task : dependent_tasks) {
    dependent_task->OnDependencyCompleted();
  }
}

}  // namespace common
}  // namespace argus

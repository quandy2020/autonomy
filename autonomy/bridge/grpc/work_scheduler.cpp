/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file work_scheduler.cpp
 * @brief Implementation of WorkScheduler ThreadPool construction and Schedule.
 */

#include "autonomy/bridge/grpc/work_scheduler.hpp"

#include "autolink/common/log.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

WorkScheduler::WorkScheduler(int num_threads, std::size_t max_tasks)
    : num_threads_(num_threads) {
    CHECK_GT(num_threads_, 0);
    pool_ = std::make_unique<::autolink::base::ThreadPool>(
        static_cast<std::size_t>(num_threads_), max_tasks);
    AINFO << "WorkScheduler started with " << num_threads_
          << " workers (queue=" << max_tasks << ")";
}

WorkScheduler::~WorkScheduler() {
    pool_.reset();
}

void WorkScheduler::Schedule(WorkItem work) {
    CHECK(pool_);
    if (!work) {
        return;
    }
    // Discard future; fire-and-forget.
    (void)pool_->Enqueue(std::move(work));
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy

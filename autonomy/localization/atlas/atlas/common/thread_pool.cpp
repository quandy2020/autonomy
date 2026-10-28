#include "atlas/common/thread_pool.hpp"

#include <algorithm>

namespace autonomy::localization::atlas {

ThreadPool::ThreadPool(std::size_t thread_count, std::size_t queue_capacity)
    : tasks_(queue_capacity) {
    if (thread_count == 0) {
        throw std::invalid_argument("ThreadPool requires at least one worker");
    }
    workers_.reserve(thread_count);
    for (std::size_t i = 0; i < thread_count; ++i) {
        workers_.emplace_back([this] { WorkerLoop(); });
    }
}

ThreadPool::~ThreadPool() {
    Stop();
}

void ThreadPool::Stop() {
    tasks_.Close();
    for (std::thread& worker : workers_) {
        if (worker.joinable())
            worker.join();
    }
    workers_.clear();
}

void ThreadPool::WorkerLoop() {
    std::function<void()> task;
    while (tasks_.Pop(&task))
        task();
}

}  // namespace autonomy::localization::atlas

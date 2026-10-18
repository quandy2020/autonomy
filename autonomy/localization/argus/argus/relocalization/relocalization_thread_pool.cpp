/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "argus/relocalization/relocalization_thread_pool.hpp"

#include <utility>

namespace argus {
namespace relocalization {

RelocalizationThreadPool::RelocalizationThreadPool(
    RelocalizationPoolConfig config)
    : config_(std::move(config)) {
    if (config_.worker_count < 1) {
        config_.worker_count = 1;
    }
    if (config_.max_queue_size < 1) {
        config_.max_queue_size = 1;
    }
}

RelocalizationThreadPool::~RelocalizationThreadPool() {
    Stop();
}

void RelocalizationThreadPool::Start() {
    if (running_.exchange(true)) {
        return;
    }
    cancel_all_.store(false);
    cancel_flags_.clear();
    cancel_flags_.reserve(static_cast<std::size_t>(config_.worker_count));
    for (int i = 0; i < config_.worker_count; ++i) {
        cancel_flags_.push_back(std::make_unique<std::atomic<bool>>(false));
    }
    workers_.clear();
    workers_.reserve(static_cast<std::size_t>(config_.worker_count));
    for (int i = 0; i < config_.worker_count; ++i) {
        workers_.emplace_back([this, i] { WorkerLoop(i); });
    }
}

void RelocalizationThreadPool::Stop() {
    if (!running_.exchange(false)) {
        return;
    }
    CancelAll();
    cv_.notify_all();
    for (auto& t : workers_) {
        if (t.joinable()) {
            t.join();
        }
    }
    workers_.clear();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
    }
}

bool RelocalizationThreadPool::Submit(Task task) {
    if (!running_.load() || !task) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (static_cast<int>(queue_.size()) >= config_.max_queue_size) {
        if (config_.on_full == RelocalizationPoolConfig::OnFull::kReject) {
            dropped_count_.fetch_add(1);
            return false;
        }
        // Drop oldest.
        if (!queue_.empty()) {
            queue_.pop_front();
            dropped_count_.fetch_add(1);
        }
    }
    queue_.push_back(std::move(task));
    cv_.notify_one();
    return true;
}

void RelocalizationThreadPool::CancelAll() {
    cancel_all_.store(true);
    for (auto& f : cancel_flags_) {
        if (f) {
            f->store(true);
        }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const int n = static_cast<int>(queue_.size());
    queue_.clear();
    if (n > 0) {
        dropped_count_.fetch_add(n);
    }
}

std::size_t RelocalizationThreadPool::queue_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}

void RelocalizationThreadPool::WorkerLoop(int worker_index) {
    while (running_.load()) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock,
                     [this] { return !running_.load() || !queue_.empty(); });
            if (!running_.load()) {
                break;
            }
            if (queue_.empty()) {
                continue;
            }
            task = std::move(queue_.front());
            queue_.pop_front();
            // Fresh cancel flag per dequeued task.
            cancel_flags_[static_cast<std::size_t>(worker_index)]->store(
                cancel_all_.load());
        }
        if (task) {
            task(*cancel_flags_[static_cast<std::size_t>(worker_index)]);
            completed_count_.fetch_add(1);
        }
    }
}

}  // namespace relocalization
}  // namespace argus

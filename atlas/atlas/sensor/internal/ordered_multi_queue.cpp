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

#include "sensor/internal/ordered_multi_queue.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include "common/logging.hpp"

namespace atlas {
namespace sensor {
namespace {

constexpr int kMaxQueueSize = 500;

}  // namespace

OrderedMultiQueue::~OrderedMultiQueue() {
  for (const auto& entry : queues_) {
    ATLAS_CHECK(entry.second.finished);
  }
}

void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  ATLAS_CHECK_EQ(queues_.count(queue_key), 0u);
  queues_[queue_key].callback = std::move(callback);
}

void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);
  ATLAS_CHECK(it != queues_.end());
  ATLAS_CHECK(!it->second.finished);
  it->second.finished = true;
  Dispatch();
}

void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  auto it = queues_.find(queue_key);
  if (it == queues_.end()) {
    return;
  }
  it->second.queue.Push(std::move(data));
  Dispatch();
}

void OrderedMultiQueue::Flush() {
  std::vector<QueueKey> unfinished;
  for (const auto& entry : queues_) {
    if (!entry.second.finished) {
      unfinished.push_back(entry.first);
    }
  }
  for (const QueueKey& key : unfinished) {
    MarkQueueAsFinished(key);
  }
}

QueueKey OrderedMultiQueue::GetBlocker() const {
  ATLAS_CHECK(!queues_.empty());
  return blocker_;
}

void OrderedMultiQueue::Dispatch() {
  while (true) {
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;
    for (auto it = queues_.begin(); it != queues_.end();) {
      const Data* data = it->second.queue.Peek();
      if (data == nullptr) {
        if (it->second.finished) {
          it = queues_.erase(it);
          continue;
        }
        CannotMakeProgress(it->first);
        return;
      }
      if (next_data == nullptr || data->GetTime() < next_data->GetTime()) {
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }
      ATLAS_CHECK_LE(last_dispatched_time_, next_data->GetTime());
      ++it;
    }
    if (next_data == nullptr) {
      return;
    }

    const double common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id);
    if (next_data->GetTime() >= common_start_time) {
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    } else if (next_queue->queue.Size() < 2) {
      if (!next_queue->finished) {
        CannotMakeProgress(next_queue_key);
        return;
      }
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    } else {
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek()->GetTime() > common_start_time) {
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(std::move(next_data_owner));
      }
    }
  }
}

void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) {
  blocker_ = queue_key;
  for (const auto& entry : queues_) {
    if (entry.second.queue.Size() > static_cast<size_t>(kMaxQueueSize)) {
      return;
    }
  }
}

double OrderedMultiQueue::GetCommonStartTime(int trajectory_id) {
  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, -std::numeric_limits<double>::infinity());
  double& common_start_time = emplace_result.first->second;
  if (emplace_result.second) {
    for (const auto& entry : queues_) {
      if (entry.first.trajectory_id == trajectory_id &&
          entry.second.queue.Peek() != nullptr) {
        common_start_time =
            std::max(common_start_time, entry.second.queue.Peek()->GetTime());
      }
    }
  }
  return common_start_time;
}

}  // namespace sensor
}  // namespace atlas

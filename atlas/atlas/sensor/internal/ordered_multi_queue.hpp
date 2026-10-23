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

#ifndef ATLAS_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_HPP_
#define ATLAS_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_HPP_

#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include "sensor/data.hpp"

namespace atlas {
namespace sensor {

struct QueueKey {
  int trajectory_id = 0;
  std::string sensor_id;

  bool operator<(const QueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) <
           std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
};

class OrderedMultiQueue {
 public:
  using Callback = std::function<void(std::unique_ptr<Data>)>;

  OrderedMultiQueue() = default;
  OrderedMultiQueue(const OrderedMultiQueue&) = delete;
  OrderedMultiQueue& operator=(const OrderedMultiQueue&) = delete;
  ~OrderedMultiQueue();

  void AddQueue(const QueueKey& queue_key, Callback callback);
  void MarkQueueAsFinished(const QueueKey& queue_key);
  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);
  void Flush();
  QueueKey GetBlocker() const;

 private:
  struct DataQueue {
    void Push(std::unique_ptr<Data> data) { items.push_back(std::move(data)); }
    std::unique_ptr<Data> Pop() {
      std::unique_ptr<Data> data = std::move(items.front());
      items.pop_front();
      return data;
    }
    const Data* Peek() const {
      return items.empty() ? nullptr : items.front().get();
    }
    size_t Size() const { return items.size(); }
    std::deque<std::unique_ptr<Data>> items;
  };

  struct Queue {
    DataQueue queue;
    Callback callback;
    bool finished = false;
  };

  void Dispatch();
  void CannotMakeProgress(const QueueKey& queue_key);
  double GetCommonStartTime(int trajectory_id);

  double last_dispatched_time_ = -1e300;
  std::map<int, double> common_start_time_per_trajectory_;
  std::map<QueueKey, Queue> queues_;
  QueueKey blocker_;
};

}  // namespace sensor
}  // namespace atlas

#endif  // ATLAS_SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_HPP_

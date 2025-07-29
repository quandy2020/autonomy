/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once 

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "autonomy/common/blocking_queue.hpp"
#include "autonomy/common/port.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/sensor/internal/dispatchable.hpp"

namespace autonomy {
namespace sensor {

struct QueueKey {
  std::string sensor_id;
};

// Maintains multiple queues of sorted sensor data and dispatches it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
//
// This class is thread-compatible.
class OrderedMultiQueue 
{
public:
    using Callback = std::function<void(std::unique_ptr<Data>)>;

    OrderedMultiQueue();
    OrderedMultiQueue(OrderedMultiQueue&& queue) = default;

    ~OrderedMultiQueue();

    // Adds a new queue with key 'queue_key' which must not already exist.
    // 'callback' will be called whenever data from this queue can be dispatched.
    void AddQueue(const QueueKey& queue_key, Callback callback);

    // Marks a queue as finished, i.e. no further data can be added. The queue
    // will be removed once the last piece of data from it has been dispatched.
    void MarkQueueAsFinished(const QueueKey& queue_key);

    // Adds 'data' to a queue with the given 'queue_key'. Data must be added
    // sorted per queue.
    void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);

    // Dispatches all remaining values in sorted order and removes the underlying
    // queues.
    void Flush();

    // Must only be called if at least one unfinished queue exists. Returns the
    // key of a queue that needs more data before the OrderedMultiQueue can
    // dispatch data.
    QueueKey GetBlocker() const;

private:
    struct Queue 
    {
        common::BlockingQueue<std::unique_ptr<Data>> queue;
        Callback callback;
        bool finished = false;
    };

    void Dispatch();

    void CannotMakeProgress(const QueueKey& queue_key);

    Time GetCommonStartTime();

    // Used to verify that values are dispatched in sorted order.
    Time last_dispatched_time_ = Time::Min();

    std::vector<Time> common_start_time_;
    std::map<QueueKey, Queue> queues_;
    QueueKey blocker_;
};

}  // namespace sensor
}  // namespace autonomy
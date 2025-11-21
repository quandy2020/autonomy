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
#include <iostream>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/blocking_queue.hpp"
#include "autonomy/tasks/command_data.hpp"

namespace autonomy {
namespace tasks { 

struct QueueKey {
    std::string task_id;
    std::size_t priority;

    bool operator<(const QueueKey& other) const {
        return std::tie(task_id, priority) < std::tie(other.task_id, other.priority);
    }

    // std::ostream& operator<<(std::ostream& os, const QueueKey& queue_key) {
    //     os << "QueueKey(" << queue_key.task_id << ", " << queue_key.priority << ")";
    //     return os;
    // }
};

class OrderedMultiCommandQueue 
{
public:
    using Callback = std::function<void(std::unique_ptr<CommandData>)>;

    OrderedMultiCommandQueue();
    OrderedMultiCommandQueue(OrderedMultiCommandQueue&& queue) = default;

    ~OrderedMultiCommandQueue();

    // Adds a new queue with key 'queue_key' which must not already exist.
    // 'callback' will be called whenever data from this queue can be dispatched.
    void AddQueue(const QueueKey& queue_key, Callback callback);

    // Marks a queue as finished, i.e. no further data can be added. The queue
    // will be removed once the last piece of data from it has been dispatched.
    void MarkQueueAsFinished(const QueueKey& queue_key);

    // Adds 'data' to a queue with the given 'queue_key'. Data must be added
    // sorted per queue.
    void Add(const QueueKey& queue_key, std::unique_ptr<CommandData> data);

    // Dispatches all remaining values in sorted order and removes the underlying
    // queues.
    void Flush();

    // Must only be called if at least one unfinished queue exists. Returns the
    // key of a queue that needs more data before the OrderedMultiQueue can
    // dispatch data.
    QueueKey GetBlocker() const;

    // Returns true if all queues are finished.
    bool IsFinished() const;

private:
    struct Queue 
    {
        autonomy::common::BlockingQueue<std::unique_ptr<CommandData>> queue;
        Callback callback;
        bool finished = false;
    };

    void Dispatch();

    void CannotMakeProgress(const QueueKey& queue_key);

    commsgs::builtin_interfaces::Time GetCommonStartTime();

    // Used to verify that values are dispatched in sorted order.
    commsgs::builtin_interfaces::Time last_dispatched_time_ = Time::Min();

    // std::vector<Time> common_start_time_;
    std::map<QueueKey, Queue> queues_;
    QueueKey blocker_;
};


}   // namespace tasks
}   // namespace autonomy
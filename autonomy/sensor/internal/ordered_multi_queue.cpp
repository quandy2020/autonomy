/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/sensor/internal/ordered_multi_queue.hpp"

#include <vector>

#include "glog/logging.h"

namespace autonomy {
namespace sensor {
namespace {

constexpr int kMaxQueueSize = 500;

}  // namespace

OrderedMultiQueue::~OrderedMultiQueue()
{
  for (const auto & entry : queues_) {
    CHECK(entry.second.finished);
  }
}

void OrderedMultiQueue::AddQueue(const QueueKey & queue_key, Callback callback)
{
  CHECK_EQ(queues_.count(queue_key), 0u);
  queues_[queue_key].callback = std::move(callback);
}

void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey & queue_key)
{
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Unknown queue '" << queue_key.sensor_id << "'";
  CHECK(!it->second.finished);
  it->second.finished = true;
  Dispatch();
}

void OrderedMultiQueue::Add(const QueueKey & queue_key, std::unique_ptr<Data> data)
{
  auto it = queues_.find(queue_key);
  if (it == queues_.end()) {
    LOG_EVERY_N(WARNING, 1000)
      << "Ignored sensor data for unregistered queue: " << queue_key.sensor_id;
    return;
  }

  if (queues_.size() == 1) {
    it->second.callback(std::move(data));
    return;
  }

  it->second.queue.Push(std::move(data));
  Dispatch();
}

void OrderedMultiQueue::Flush()
{
  std::vector<QueueKey> unfinished;
  for (const auto & entry : queues_) {
    if (!entry.second.finished) {
      unfinished.push_back(entry.first);
    }
  }
  for (const auto & key : unfinished) {
    MarkQueueAsFinished(key);
  }
}

QueueKey OrderedMultiQueue::GetBlocker() const
{
  CHECK(!queues_.empty());
  return blocker_;
}

Time OrderedMultiQueue::GetCommonStartTime()
{
  if (!common_start_ready_) {
    common_start_ready_ = true;
    common_start_time_ = Time{};
    for (const auto & entry : queues_) {
      const Data * peek = entry.second.queue.Peek<Data>();
      if (peek != nullptr) {
        common_start_time_ = std::max(common_start_time_, peek->GetTime());
      }
    }
  }
  return common_start_time_;
}

void OrderedMultiQueue::Dispatch()
{
  while (true) {
    const Data * next_data = nullptr;
    Queue * next_queue = nullptr;
    QueueKey next_queue_key;
    for (auto it = queues_.begin(); it != queues_.end();) {
      const Data * data = it->second.queue.Peek<Data>();
      if (data == nullptr) {
        if (it->second.finished) {
          queues_.erase(it++);
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
      CHECK_LE(last_dispatched_time_, next_data->GetTime())
        << "Non-sorted data on queue '" << it->first.sensor_id << "'";
      ++it;
    }
    if (next_data == nullptr) {
      CHECK(queues_.empty());
      return;
    }

    const Time common_start_time = GetCommonStartTime();
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
      std::unique_ptr<Data> owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->GetTime() > common_start_time) {
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(std::move(owner));
      }
    }
  }
}

void OrderedMultiQueue::CannotMakeProgress(const QueueKey & queue_key)
{
  blocker_ = queue_key;
  for (const auto & entry : queues_) {
    if (entry.second.queue.Size() > static_cast<size_t>(kMaxQueueSize)) {
      LOG_EVERY_N(WARNING, 60)
        << "Sensor queue waiting for data: " << queue_key.sensor_id;
      return;
    }
  }
}

}  // namespace sensor
}  // namespace autonomy

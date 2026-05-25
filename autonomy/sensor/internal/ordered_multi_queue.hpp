/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "autonomy/common/blocking_queue.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/sensor/data.hpp"

namespace autonomy {
namespace sensor {

struct QueueKey
{
  std::string sensor_id;

  bool operator<(const QueueKey & other) const
  {
    return sensor_id < other.sensor_id;
  }
};

/** @brief Merge-sorted dispatch across sensor queues. */
class OrderedMultiQueue
{
public:
  using Callback = std::function<void(std::unique_ptr<Data>)>;

  OrderedMultiQueue() = default;
  OrderedMultiQueue(const OrderedMultiQueue &) = delete;
  OrderedMultiQueue & operator=(const OrderedMultiQueue &) = delete;
  ~OrderedMultiQueue();

  void AddQueue(const QueueKey & queue_key, Callback callback);
  void MarkQueueAsFinished(const QueueKey & queue_key);
  void Add(const QueueKey & queue_key, std::unique_ptr<Data> data);
  void Flush();
  QueueKey GetBlocker() const;

private:
  struct Queue
  {
    common::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;
    bool finished{false};
  };

  void Dispatch();
  void CannotMakeProgress(const QueueKey & queue_key);
  Time GetCommonStartTime();

  Time last_dispatched_time_{};
  bool common_start_ready_{false};
  Time common_start_time_{};
  std::map<QueueKey, Queue> queues_;
  QueueKey blocker_;
};

}  // namespace sensor
}  // namespace autonomy

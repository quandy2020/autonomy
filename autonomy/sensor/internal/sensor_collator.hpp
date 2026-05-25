/*
 * Copyright 2026 autonomy contributors
 */

#pragma once

#include <vector>

#include "autonomy/sensor/collator_interface.hpp"
#include "autonomy/sensor/consumer.hpp"
#include "autonomy/sensor/internal/ordered_multi_queue.hpp"

namespace autonomy {
namespace sensor {

/** @brief Merge-sorted sensor ingress; odometry bypasses the queue. */
class SensorCollator : public CollatorInterface
{
public:
  explicit SensorCollator(SensorConsumer consumer);

  void SetDispatchCallback(const Callback & callback) override;
  void AddSensorData(std::unique_ptr<Data> data) override;
  void Flush() override;

private:
  void EnsureQueue(const std::string & sensor_id);

  SensorConsumer consumer_;
  Callback dispatch_callback_;
  OrderedMultiQueue queue_;
  std::vector<QueueKey> queue_keys_;
};

}  // namespace sensor
}  // namespace autonomy

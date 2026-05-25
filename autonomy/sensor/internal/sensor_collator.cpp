/*
 * Copyright 2026 autonomy contributors
 */

#include "autonomy/sensor/internal/sensor_collator.hpp"

#include <algorithm>

#include "glog/logging.h"

namespace autonomy {
namespace sensor {

SensorCollator::SensorCollator(SensorConsumer consumer)
: consumer_(std::move(consumer))
{
}

void SensorCollator::SetDispatchCallback(const Callback & callback)
{
  dispatch_callback_ = callback;
}

void SensorCollator::EnsureQueue(const std::string & sensor_id)
{
  const QueueKey key{sensor_id};
  if (std::find_if(
        queue_keys_.begin(), queue_keys_.end(),
        [&](const QueueKey & k) { return k.sensor_id == sensor_id; }) != queue_keys_.end()) {
    return;
  }
  if (!dispatch_callback_) {
    return;
  }
  const Callback & dispatch = dispatch_callback_;
  queue_.AddQueue(
    key,
    [dispatch, sensor_id](std::unique_ptr<Data> data) {
      dispatch(sensor_id, std::move(data));
    });
  queue_keys_.push_back(key);
}

void SensorCollator::AddSensorData(std::unique_ptr<Data> data)
{
  if (!data) {
    return;
  }

  if (data->GetType() == SensorDataType::kOdometry) {
    data->Dispatch(consumer_);
    return;
  }

  if (!dispatch_callback_) {
    LOG_EVERY_N(WARNING, 100) << "Sensor collator has no dispatch callback";
    return;
  }
  EnsureQueue(data->GetSensorId());
  queue_.Add(QueueKey{data->GetSensorId()}, std::move(data));
}

void SensorCollator::Flush()
{
  for (const auto & key : queue_keys_) {
    queue_.MarkQueueAsFinished(key);
  }
}

}  // namespace sensor
}  // namespace autonomy

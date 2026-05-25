/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/time.hpp"

namespace autonomy {
namespace sensor {

class SensorConsumer;

enum class SensorDataType
{
  kOdometry,
  kLaserScan,
  kPointCloud2,
  kRange
};

using Time = common::Time;

class Data
{
public:
  explicit Data(std::string sensor_id) : sensor_id_(std::move(sensor_id)) {}
  virtual ~Data() = default;

  virtual SensorDataType GetType() const = 0;
  virtual Time GetTime() const = 0;
  const std::string & GetSensorId() const { return sensor_id_; }

  virtual void Dispatch(SensorConsumer & consumer) const = 0;

protected:
  std::string sensor_id_;
};

}  // namespace sensor
}  // namespace autonomy

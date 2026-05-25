/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <functional>
#include <memory>

#include "autonomy/sensor/data.hpp"

namespace autonomy {
namespace sensor {

class CollatorInterface
{
public:
  using Callback =
    std::function<void(const std::string & sensor_id, std::unique_ptr<Data> data)>;

  CollatorInterface() = default;
  virtual ~CollatorInterface() = default;

  CollatorInterface(const CollatorInterface &) = delete;
  CollatorInterface & operator=(const CollatorInterface &) = delete;

  virtual void SetDispatchCallback(const Callback & callback) = 0;

  /** Per-sensor streams must be time-ordered. Odometry is dispatched immediately. */
  virtual void AddSensorData(std::unique_ptr<Data> data) = 0;

  virtual void Flush() = 0;
};

}  // namespace sensor
}  // namespace autonomy

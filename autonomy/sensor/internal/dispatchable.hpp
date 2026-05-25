/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <memory>
#include <string>
#include <utility>

#include "autonomy/sensor/consumer.hpp"
#include "autonomy/sensor/data.hpp"
#include "autonomy/sensor/laser_data.hpp"
#include "autonomy/sensor/odometry_data.hpp"
#include "autonomy/sensor/point_cloud.hpp"
#include "autonomy/sensor/range_data.hpp"
#include "autonomy/sensor/time_util.hpp"

namespace autonomy {
namespace sensor {

template <SensorDataType kType, typename Payload>
class Dispatchable : public Data
{
public:
  Dispatchable(std::string sensor_id, Payload payload)
  : Data(std::move(sensor_id)), payload_(std::move(payload)) {}

  SensorDataType GetType() const override { return kType; }

  Time GetTime() const override { return TimeFromHeader(payload_.header); }

  void Dispatch(SensorConsumer & consumer) const override
  {
    DispatchImpl(consumer);
  }

  const Payload & payload() const { return payload_; }

private:
  void DispatchImpl(SensorConsumer & consumer) const;

  Payload payload_;
};

template <>
inline void Dispatchable<SensorDataType::kOdometry, OdometryData>::DispatchImpl(
  SensorConsumer & consumer) const
{
  consumer.OnOdometry(payload_);
}

template <>
inline void Dispatchable<SensorDataType::kLaserScan, LaserScanData>::DispatchImpl(
  SensorConsumer & consumer) const
{
  consumer.OnLaserScan(payload_);
}

template <>
inline void Dispatchable<SensorDataType::kPointCloud2, PointCloud2Data>::DispatchImpl(
  SensorConsumer & consumer) const
{
  consumer.OnPointCloud2(payload_);
}

template <>
inline void Dispatchable<SensorDataType::kRange, RangeData>::DispatchImpl(
  SensorConsumer & consumer) const
{
  consumer.OnRange(payload_);
}

template <SensorDataType kType, typename Payload>
std::unique_ptr<Data> MakeDispatchable(std::string sensor_id, Payload payload)
{
  return std::make_unique<Dispatchable<kType, Payload>>(
    std::move(sensor_id), std::move(payload));
}

inline std::string DefaultSensorId(const std::string & frame_id, const char * fallback)
{
  return frame_id.empty() ? fallback : frame_id;
}

inline std::unique_ptr<Data> MakeOdometryData(
  OdometryData odom, std::string sensor_id = {})
{
  return MakeDispatchable<SensorDataType::kOdometry, OdometryData>(
    DefaultSensorId(sensor_id.empty() ? odom.header.frame_id : sensor_id, "odom"),
    std::move(odom));
}

inline std::unique_ptr<Data> MakeLaserScanData(
  LaserScanData scan, std::string sensor_id = {})
{
  return MakeDispatchable<SensorDataType::kLaserScan, LaserScanData>(
    DefaultSensorId(sensor_id.empty() ? scan.header.frame_id : sensor_id, "laser"),
    std::move(scan));
}

inline std::unique_ptr<Data> MakePointCloud2Data(
  PointCloud2Data cloud, std::string sensor_id = {})
{
  return MakeDispatchable<SensorDataType::kPointCloud2, PointCloud2Data>(
    DefaultSensorId(sensor_id.empty() ? cloud.header.frame_id : sensor_id, "point_cloud"),
    std::move(cloud));
}

inline std::unique_ptr<Data> MakeRangeData(RangeData range, std::string sensor_id = {})
{
  return MakeDispatchable<SensorDataType::kRange, RangeData>(
    DefaultSensorId(sensor_id.empty() ? range.header.frame_id : sensor_id, "range"),
    std::move(range));
}

}  // namespace sensor
}  // namespace autonomy

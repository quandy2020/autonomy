/*
 * Copyright 2026 Autodriver contributors
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

/**
 * @file
 * @brief Implements mock sensor drivers.
 */

#include "autodriver/drivers/mock/mock_drivers.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "autodriver/common/time.hpp"
#include "autodriver/types/camera_frame.hpp"
#include "autodriver/types/gps_sample.hpp"
#include "autodriver/types/imu_sample.hpp"
#include "autodriver/types/lidar_scan.hpp"
#include "autodriver/types/range_sample.hpp"
#include "autodriver/types/wheel_odometry_sample.hpp"

namespace autodriver {
namespace mock {
namespace {

class PeriodicMockDriver : public SensorDriver
{
public:
  PeriodicMockDriver(
    SensorId sensor_id,
    SensorType type,
    std::chrono::milliseconds period,
    std::function<std::unique_ptr<SensorSample>(const SensorId &, Timestamp)>
    generator)
  : sensor_id_(std::move(sensor_id)),
    type_(type),
    period_(period),
    generator_(std::move(generator))
  {
  }

  ~PeriodicMockDriver() override { Stop(); }

  SensorType GetType() const override { return type_; }
  const SensorId & GetSensorId() const override { return sensor_id_; }

  bool Start() override
  {
    if (running_.exchange(true)) {
      return true;
    }
    worker_ = std::thread([this]() { RunLoop(); });
    return true;
  }

  void Stop() override
  {
    if (!running_.exchange(false)) {
      return;
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  bool IsRunning() const override { return running_.load(); }

  void SetSampleCallback(SampleCallback callback) override
  {
    callback_ = std::move(callback);
  }

private:
  void RunLoop()
  {
    std::size_t tick = 0;
    while (running_.load()) {
      if (callback_) {
        callback_(generator_(sensor_id_, Now()));
      }
      ++tick;
      std::this_thread::sleep_for(period_);
    }
  }

  SensorId sensor_id_;
  SensorType type_;
  std::chrono::milliseconds period_;
  std::function<std::unique_ptr<SensorSample>(const SensorId &, Timestamp)>
  generator_;
  SampleCallback callback_;
  std::atomic<bool> running_{false};
  std::thread worker_;
};

std::unique_ptr<SensorSample> MakeMockImu(
  const SensorId & id, Timestamp time, std::size_t tick)
{
  const double phase = static_cast<double>(tick) * 0.1;
  return std::make_unique<ImuSample>(
    id, time,
    std::array<double, 3>{0.0, 0.0, 0.05 * std::sin(phase)},
    std::array<double, 3>{0.0, 0.0, 9.81});
}

std::unique_ptr<SensorSample> MakeMockGps(const SensorId & id, Timestamp time)
{
  return std::make_unique<GpsSample>(
    id, time, 31.2304, 121.4737, 4.0, GpsFixStatus::kFix3D);
}

std::unique_ptr<SensorSample> MakeMockWheelOdom(
  const SensorId & id, Timestamp time, std::size_t tick)
{
  const double phase = static_cast<double>(tick) * 0.05;
  return std::make_unique<WheelOdometrySample>(
    id, time, 0.5 + 0.1 * std::sin(phase), 0.0, 0.02 * std::cos(phase));
}

std::unique_ptr<SensorSample> MakeMockLidar(
  const SensorId & id, Timestamp time, std::size_t tick)
{
  std::vector<float> ranges(360);
  std::vector<float> angles(360);
  const float base = 5.0f + 0.5f * std::sin(static_cast<float>(tick) * 0.1f);
  const float kPi = 3.14159265f;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    angles[i] = static_cast<float>(i) * kPi / 180.0f;
    ranges[i] = base;
  }
  return std::make_unique<LidarScan>(id, time, ranges, angles, 0.1f, 30.0f);
}

std::unique_ptr<SensorSample> MakeMockRange(
  const SensorId & id, Timestamp time, std::size_t tick)
{
  const double range = 1.0 + 0.2 * std::sin(static_cast<double>(tick) * 0.2);
  return std::make_unique<RangeSample>(id, time, range, 0.02, 4.0);
}

std::unique_ptr<SensorSample> MakeMockCamera(
  const SensorId & id, Timestamp time, std::size_t tick)
{
  constexpr std::uint32_t kWidth = 8;
  constexpr std::uint32_t kHeight = 8;
  std::vector<std::uint8_t> data(kWidth * kHeight * 3,
    static_cast<std::uint8_t>(tick % 256));
  return std::make_unique<CameraFrame>(
    id, time, kWidth, kHeight, "rgb8", std::move(data));
}

template<typename FactoryFn>
std::shared_ptr<SensorDriver> CreatePeriodicDriver(
  SensorId sensor_id,
  SensorType type,
  std::chrono::milliseconds period,
  FactoryFn factory)
{
  static std::atomic<std::size_t> global_tick{0};
  return std::make_shared<PeriodicMockDriver>(
    std::move(sensor_id), type, period,
    [factory](const SensorId & id, Timestamp time) {
      return factory(id, time, global_tick.fetch_add(1));
    });
}

}  // namespace

void RegisterMockDrivers()
{
  auto & factory = SensorFactory::Instance();
  factory.Register("mock_imu", [] { return CreateMockImuDriver(); });
  factory.Register("mock_gps", [] { return CreateMockGpsDriver(); });
  factory.Register("mock_wheel_odom", [] { return CreateMockWheelOdometryDriver(); });
  factory.Register("mock_lidar", [] { return CreateMockLidarDriver(); });
  factory.Register("mock_range", [] { return CreateMockRangeFinderDriver(); });
  factory.Register("mock_camera", [] { return CreateMockCameraDriver(); });
}

std::shared_ptr<SensorDriver> CreateMockImuDriver(
  SensorId sensor_id,
  std::chrono::milliseconds period)
{
  return CreatePeriodicDriver(
    std::move(sensor_id), SensorType::kImu, period, MakeMockImu);
}

std::shared_ptr<SensorDriver> CreateMockGpsDriver(
  SensorId sensor_id,
  std::chrono::milliseconds period)
{
  return CreatePeriodicDriver(
    std::move(sensor_id), SensorType::kGps, period,
    [](const SensorId & id, Timestamp time, std::size_t) {
      return MakeMockGps(id, time);
    });
}

std::shared_ptr<SensorDriver> CreateMockWheelOdometryDriver(
  SensorId sensor_id,
  std::chrono::milliseconds period)
{
  return CreatePeriodicDriver(
    std::move(sensor_id), SensorType::kWheelOdometry, period, MakeMockWheelOdom);
}

std::shared_ptr<SensorDriver> CreateMockLidarDriver(
  SensorId sensor_id,
  std::chrono::milliseconds period)
{
  return CreatePeriodicDriver(
    std::move(sensor_id), SensorType::kLidar, period, MakeMockLidar);
}

std::shared_ptr<SensorDriver> CreateMockRangeFinderDriver(
  SensorId sensor_id,
  std::chrono::milliseconds period)
{
  return CreatePeriodicDriver(
    std::move(sensor_id), SensorType::kRangeFinder, period, MakeMockRange);
}

std::shared_ptr<SensorDriver> CreateMockCameraDriver(
  SensorId sensor_id,
  std::chrono::milliseconds period)
{
  return CreatePeriodicDriver(
    std::move(sensor_id), SensorType::kCamera, period, MakeMockCamera);
}

std::shared_ptr<SensorDriver> CreateByName(
  const std::string & factory_name,
  const SensorId & sensor_id)
{
  if (factory_name == "mock_imu") {
    return CreateMockImuDriver(
      sensor_id.empty() ? SensorId{"imu/mock"} : sensor_id);
  }
  if (factory_name == "mock_gps") {
    return CreateMockGpsDriver(
      sensor_id.empty() ? SensorId{"gps/mock"} : sensor_id);
  }
  if (factory_name == "mock_wheel_odom") {
    return CreateMockWheelOdometryDriver(
      sensor_id.empty() ? SensorId{"wheel_odom/mock"} : sensor_id);
  }
  if (factory_name == "mock_lidar") {
    return CreateMockLidarDriver(
      sensor_id.empty() ? SensorId{"lidar/mock"} : sensor_id);
  }
  if (factory_name == "mock_range") {
    return CreateMockRangeFinderDriver(
      sensor_id.empty() ? SensorId{"range/mock"} : sensor_id);
  }
  if (factory_name == "mock_camera") {
    return CreateMockCameraDriver(
      sensor_id.empty() ? SensorId{"camera/mock"} : sensor_id);
  }
  return SensorFactory::Instance().Create(factory_name);
}

}  // namespace mock
}  // namespace autodriver

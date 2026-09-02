/*
 * Copyright 2026 Autodriver contributors
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "autodriver/driver_params.hpp"
#include "autodriver/hardware/realsense_device_hub.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {
namespace hardware {

std::shared_ptr<SensorDriver> CreateRealSensePointCloudDriver(
    const SensorId& id, const DriverParams& params);

class RealSensePointCloudDriver : public SensorDriver {
 public:
  RealSensePointCloudDriver(SensorId id, DriverParams params);
  ~RealSensePointCloudDriver() override;

  SensorType GetType() const override { return SensorType::kLidar3d; }
  const SensorId& GetSensorId() const override { return id_; }
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override;
  void SetSampleCallback(SampleCallback callback) override;

 private:
    SensorId id_;
    DriverParams params_;
    int width_{640};
    int height_{480};
    int fps_{30};
    std::atomic<bool> running_{false};
    std::shared_ptr<io::RealSenseDeviceHub> hub_;
    std::uint64_t subscription_id_{0};
    SampleCallback callback_;
};

}  // namespace hardware
}  // namespace autodriver

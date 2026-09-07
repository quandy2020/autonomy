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
 * @brief SocketCAN GNSS driver (NMEA2000 lat/lon frame).
 */

#ifndef AUTODRIVER_GPS_CAN_GPS_DRIVER_HPP_
#define AUTODRIVER_GPS_CAN_GPS_DRIVER_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "autodriver/canbus/can_receiver.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"

namespace autodriver {
namespace hardware {

/**
 * @brief Lat/lon fix decoded from one CAN frame (degrees).
 */
struct GpsCanFix {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
};

/**
 * @class autodriver::hardware::CanGpsDriver
 * @brief Decodes lat/lon via CanReceiver + MessageManager (NMEA2000 PGN 129025).
 *
 * Params: `interface` (e.g. can0), `can_id` (default 0x12902500).
 */
class CanGpsDriver : public SensorDriver {
public:
    CanGpsDriver(SensorId id, DriverParams params);
    ~CanGpsDriver() override;

    SensorType GetType() const override { return SensorType::kGps; }
    const SensorId& GetSensorId() const override { return id_; }

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;
    void SetSampleCallback(SampleCallback callback) override;

private:
    /** Publish NavSatFix from a decoded lat/lon frame. */
    void OnFix(const GpsCanFix& fix);

    SensorId id_;
    DriverParams params_;
    // Expected CAN id for the lat/lon frame.
    std::uint32_t can_id_{0};
    canbus::CanReceiver<GpsCanFix> receiver_;
    SampleCallback callback_;
};

/**
 * @brief Factory for CanGpsDriver (used by GpsModule backend "can").
 */
std::shared_ptr<SensorDriver> CreateCanGpsDriver(const SensorId& id,
                                                 const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_GPS_CAN_GPS_DRIVER_HPP_

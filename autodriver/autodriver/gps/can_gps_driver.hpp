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

#include "autodriver/common/can_sensor_driver_base.hpp"
#include "autodriver/driver_params.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace hardware {

/**
 * @struct autodriver::hardware::GpsCanFix
 * @brief Lat/lon fix decoded from one CAN frame (degrees).
 */
struct GpsCanFix {
    /** @brief Latitude in degrees. */
    double latitude_deg = 0.0;
    /** @brief Longitude in degrees. */
    double longitude_deg = 0.0;
};

/**
 * @class autodriver::hardware::CanGpsDriver
 * @brief Decodes lat/lon via CanSensorDriverBase (NMEA2000 PGN 129025).
 *
 * Params: `interface` (e.g. can0), `can_id` (default 0x12902500).
 */
class CanGpsDriver : public CanSensorDriverBase<CanGpsDriver, GpsCanFix> {
public:
    /**
     * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(CanGpsDriver)

    /**
     * @brief Register lat/lon ProtocolData and publish callback.
     * @param id Sensor instance id.
     * @param params DriverParams (cold-path parse only).
     */
    CanGpsDriver(SensorId id, DriverParams params);

    /**
     * @brief Stops the CAN receive loop.
     */
    ~CanGpsDriver() override { Stop(); }

    /**
     * @brief Report sensor type.
     * @return SensorType::kGps.
     */
    SensorType GetSensorType() const override { return SensorType::kGps; }

private:
    /**
     * @brief Publish NavSatFix from a decoded lat/lon frame.
     * @param fix Decoded fix from MessageManager.
     */
    void OnFix(const GpsCanFix& fix);

    // Expected CAN id for the lat/lon frame.
    std::uint32_t can_id_{0};
};

/**
 * @brief Factory for GpsBackendRegistry (REGISTER_GPS_BACKEND "can").
 * @param id Sensor instance id from YAML.
 * @param params Backend-specific key/value map.
 * @return Owning CanGpsDriver* (never null).
 */
SensorDriver* CreateCanGpsDriver(const SensorId& id,
                                 const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_GPS_CAN_GPS_DRIVER_HPP_

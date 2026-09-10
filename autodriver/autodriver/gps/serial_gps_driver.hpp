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
 * @brief Serial NMEA GNSS receiver driver.
 */

#ifndef AUTODRIVER_GPS_SERIAL_GPS_DRIVER_HPP_
#define AUTODRIVER_GPS_SERIAL_GPS_DRIVER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "autodriver/common/serial_byte_driver_base.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/gps/parser/parser.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::SerialGpsDriver
 * @brief Reads NMEA via SerialByteDriverBase + gps::GnssParser ("nmea").
 *
 * Required params: `device` (default `/dev/ttyUSB0`). Optional: `baud` (115200).
 */
class SerialGpsDriver : public SerialByteDriverBase<SerialGpsDriver> {
public:
    /**
     * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(SerialGpsDriver)

    /**
     * @brief Store sensor identity and serial driver params.
     * @param id Sensor instance id.
     * @param params DriverParams (cold-path parse only).
     */
    SerialGpsDriver(SensorId id, DriverParams params);

    /**
     * @brief Stops the reader thread before destroying the NMEA parser.
     */
    ~SerialGpsDriver() override { Stop(); }

    /**
     * @brief Report sensor type.
     * @return SensorType::kGps.
     */
    SensorType GetSensorType() const override { return SensorType::kGps; }

    /**
     * @brief CRTP hook: create NMEA parser before opening the stream.
     * @return true when GnssParserRegistry yields a parser.
     */
    bool PrepareStart();

    /**
     * @brief CRTP hook: release the NMEA parser after Stop().
     */
    void OnStopped();

    /**
     * @brief CRTP hook: feed NMEA parser and emit GpsSample for each fix.
     * @param data Bytes read from the serial Stream.
     * @param n Number of valid bytes in @p data.
     */
    void OnBytes(const std::uint8_t* data, std::size_t n);

private:
    // Streaming NMEA parser (line buffer lives inside the parser).
    std::unique_ptr<gps::GnssParser> parser_{nullptr};
};

/**
 * @brief Factory for GpsBackendRegistry (REGISTER_GPS_BACKEND "serial").
 * @param id Sensor instance id from YAML.
 * @param params Backend-specific key/value map.
 * @return Owning SerialGpsDriver* (never null).
 */
SensorDriver* CreateSerialGpsDriver(const SensorId& id,
                                    const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_GPS_SERIAL_GPS_DRIVER_HPP_

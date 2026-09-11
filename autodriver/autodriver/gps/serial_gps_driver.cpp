/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file serial_gps_driver.cpp
 * @brief Serial NMEA GNSS receiver driver (implementation).
 */

#include "autodriver/gps/serial_gps_driver.hpp"

#include <utility>

#include "autodriver/types/sensor_sample.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {
namespace hardware {

SerialGpsDriver::SerialGpsDriver(SensorId id, DriverParams params)
    : SerialByteDriverBase<SerialGpsDriver>(std::move(id), std::move(params),
                                            /*read_timeout_ms=*/100) {}

bool SerialGpsDriver::PrepareStart() {
    parser_ = gps::GnssParserRegistry::Instance().CreateParser("nmea");
    return static_cast<bool>(parser_);
}

void SerialGpsDriver::OnStopped() { parser_.reset(); }

void SerialGpsDriver::OnBytes(const std::uint8_t* data, std::size_t n) {
    if (!parser_) {
        return;
    }
    auto fix = parser_->Consume(data, n);
    while (fix) {
        EmitSample(std::make_unique<GpsSample>(
            id_, autolink::Time::Now(),
            GpsMsg(fix->latitude_deg, fix->longitude_deg, fix->altitude_m,
                   fix->status)));
        fix = parser_->Consume(nullptr, 0);
    }
}

SensorDriver* CreateSerialGpsDriver(const SensorId& id,
                                    const DriverParams& params) {
    return new SerialGpsDriver(id, params);
}

}  // namespace hardware
}  // namespace autodriver

#include "autodriver/gps/backend_register.hpp"

REGISTER_GPS_BACKEND(serial, "serial",
                     autodriver::hardware::CreateSerialGpsDriver, "");

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
 * @file can_gps_driver.cpp
 * @brief SocketCAN GNSS driver (NMEA2000 lat/lon frame) (implementation).
 */

#include "autodriver/gps/can_gps_driver.hpp"

#include <utility>

#include "autodriver/imu/wit_motion_parser.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {
namespace hardware {
namespace {

class LatLonProtocol : public canbus::ProtocolData<GpsCanFix> {
public:
    explicit LatLonProtocol(std::uint32_t can_id) : can_id_(can_id) {}

    std::uint32_t can_id() const override { return can_id_; }

    bool Parse(const io::CanFrame& frame, GpsCanFix* msg) const override {
        if (msg == nullptr) {
            return false;
        }
        const auto fix =
            protocol::ParseNmea2000LatLonFrame(frame.data, frame.dlc);
        if (!fix) {
            return false;
        }
        msg->latitude_deg = fix->latitude_deg;
        msg->longitude_deg = fix->longitude_deg;
        return true;
    }

private:
    std::uint32_t can_id_ = 0;
};

}  // namespace

CanGpsDriver::CanGpsDriver(SensorId id, DriverParams params)
    : CanSensorDriverBase<CanGpsDriver, GpsCanFix>(std::move(id),
                                                   std::move(params), 100),
      can_id_(ParseCanId(params_, "can_id", 0x12902500)) {
    receiver().manager().Register(std::make_shared<LatLonProtocol>(can_id_));
    receiver().manager().SetPublishCallback(
        [this](const GpsCanFix& fix) { OnFix(fix); });
}

void CanGpsDriver::OnFix(const GpsCanFix& fix) {
    EmitSample(std::make_unique<GpsSample>(
        id_, autolink::Time::Now(),
        GpsMsg(fix.latitude_deg, fix.longitude_deg, 0.0,
               automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_FIX)));
}

SensorDriver* CreateCanGpsDriver(const SensorId& id,
                                 const DriverParams& params) {
    return new CanGpsDriver(id, params);
}

}  // namespace hardware
}  // namespace autodriver

#include "autodriver/gps/backend_register.hpp"

REGISTER_GPS_BACKEND(can, "can", autodriver::hardware::CreateCanGpsDriver, "");

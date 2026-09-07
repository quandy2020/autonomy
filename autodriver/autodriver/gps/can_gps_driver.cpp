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
    : id_(std::move(id)),
      params_(std::move(params)),
      can_id_(ParseCanId(params_, "can_id", 0x12902500)) {
    receiver_.manager().Register(std::make_shared<LatLonProtocol>(can_id_));
    receiver_.manager().SetPublishCallback(
        [this](const GpsCanFix& fix) { OnFix(fix); });
}

CanGpsDriver::~CanGpsDriver() { Stop(); }

bool CanGpsDriver::Start() {
    const std::string interface_name = GetString(params_, "interface", "can0");
    return receiver_.Start(interface_name, 100);
}

void CanGpsDriver::Stop() { receiver_.Stop(); }

bool CanGpsDriver::IsRunning() const { return receiver_.IsRunning(); }

void CanGpsDriver::SetSampleCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

void CanGpsDriver::OnFix(const GpsCanFix& fix) {
    if (!callback_) {
        return;
    }
    callback_(std::make_unique<GpsSample>(
        id_, autolink::Time::Now(),
        GpsMsg(fix.latitude_deg, fix.longitude_deg, 0.0,
               automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_FIX)));
}

std::shared_ptr<SensorDriver> CreateCanGpsDriver(const SensorId& id,
                                                 const DriverParams& params) {
    return std::make_shared<CanGpsDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

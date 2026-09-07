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

#include "autodriver/imu/can_imu_driver.hpp"

#include <utility>

#include "autodriver/imu/wit_motion_parser.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {
namespace hardware {
namespace {

class AccelProtocol : public canbus::ProtocolData<ImuCanEvent> {
public:
    AccelProtocol(std::uint32_t can_id, double scale)
        : can_id_(can_id), scale_(scale) {}

    std::uint32_t can_id() const override { return can_id_; }

    bool Parse(const io::CanFrame& frame, ImuCanEvent* msg) const override {
        if (frame.dlc < 6 || msg == nullptr) {
            return false;
        }
        msg->kind = ImuCanEvent::Kind::kAccel;
        msg->values = protocol::DecodeScaledInt16Triplet(frame.data, scale_);
        return true;
    }

private:
    std::uint32_t can_id_ = 0;
    double scale_ = 0.001;
};

class GyroProtocol : public canbus::ProtocolData<ImuCanEvent> {
public:
    GyroProtocol(std::uint32_t can_id, double scale)
        : can_id_(can_id), scale_(scale) {}

    std::uint32_t can_id() const override { return can_id_; }

    bool Parse(const io::CanFrame& frame, ImuCanEvent* msg) const override {
        if (frame.dlc < 6 || msg == nullptr) {
            return false;
        }
        msg->kind = ImuCanEvent::Kind::kGyro;
        msg->values = protocol::DecodeScaledInt16Triplet(frame.data, scale_);
        return true;
    }

private:
    std::uint32_t can_id_ = 0;
    double scale_ = 0.0001;
};

}  // namespace

CanImuDriver::CanImuDriver(SensorId id, DriverParams params)
    : id_(std::move(id)),
      params_(std::move(params)),
      accel_can_id_(ParseCanId(params_, "accel_can_id", 0x100)),
      gyro_can_id_(ParseCanId(params_, "gyro_can_id", 0x101)),
      accel_scale_(ParseDouble(params_, "accel_scale", 0.001)),
      gyro_scale_(ParseDouble(params_, "gyro_scale", 0.0001)) {
    receiver_.manager().Register(
        std::make_shared<AccelProtocol>(accel_can_id_, accel_scale_));
    receiver_.manager().Register(
        std::make_shared<GyroProtocol>(gyro_can_id_, gyro_scale_));
    receiver_.manager().SetPublishCallback(
        [this](const ImuCanEvent& event) { OnEvent(event); });
}

CanImuDriver::~CanImuDriver() { Stop(); }

bool CanImuDriver::Start() {
    const std::string interface_name = GetString(params_, "interface", "can0");
    return receiver_.Start(interface_name, 100);
}

void CanImuDriver::Stop() { receiver_.Stop(); }

bool CanImuDriver::IsRunning() const { return receiver_.IsRunning(); }

void CanImuDriver::SetSampleCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

void CanImuDriver::OnEvent(const ImuCanEvent& event) {
    if (event.kind == ImuCanEvent::Kind::kAccel) {
        accel_ = event.values;
        have_accel_ = true;
    } else {
        gyro_ = event.values;
        have_gyro_ = true;
    }
    TryEmit();
}

void CanImuDriver::TryEmit() {
    if (!have_accel_ || !have_gyro_ || !callback_) {
        return;
    }
    callback_(std::make_unique<ImuSample>(
        id_, autolink::Time::Now(), ImuMsg(gyro_, accel_)));
    have_accel_ = false;
    have_gyro_ = false;
}

std::shared_ptr<SensorDriver> CreateCanImuDriver(const SensorId& id,
                                                 const DriverParams& params) {
    return std::make_shared<CanImuDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

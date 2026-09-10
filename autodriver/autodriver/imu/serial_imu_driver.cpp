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

#include "autodriver/imu/serial_imu_driver.hpp"

#include <utility>

#include "autodriver/types/sensor_sample.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {
namespace hardware {

SerialImuDriver::SerialImuDriver(SensorId id, DriverParams params)
    : SerialByteDriverBase<SerialImuDriver>(std::move(id), std::move(params),
                                            /*read_timeout_ms=*/50),
      parser_(ParseDouble(params_, "accel_scale", 16.0 * 9.80665 / 32768.0),
              ParseDouble(params_, "gyro_scale",
                          2000.0 * 3.141592653589793 / 180.0 / 32768.0)) {}

void SerialImuDriver::OnBytes(const std::uint8_t* data, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        parser_.Feed(data[i]);
        if (!parser_.HasCompleteSample()) {
            continue;
        }
        const auto& state = parser_.state();
        EmitSample(std::make_unique<ImuSample>(
            id_, autolink::Time::Now(),
            ImuMsg(state.angular_velocity, state.linear_acceleration)));
        parser_.ResetSampleFlags();
    }
}

SensorDriver* CreateSerialImuDriver(const SensorId& id,
                                    const DriverParams& params) {
    return new SerialImuDriver(id, params);
}

}  // namespace hardware
}  // namespace autodriver

#include "autodriver/imu/backend_register.hpp"

REGISTER_IMU_BACKEND(serial, "serial",
                     autodriver::hardware::CreateSerialImuDriver, "");

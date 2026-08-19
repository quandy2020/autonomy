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

#include "autodriver/hardware/can_gps_driver.hpp"
#include "autodriver/hardware/serial_gps_driver.hpp"
#include "autodriver/sensor_plugin.hpp"
#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"

class GpsModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kGps> {
protected:
    std::shared_ptr<autodriver::SensorDriver> MakeDriver(
        const autodriver::Config::Sensor& sensor) override {
        if (sensor.backend == "serial") {
            return autodriver::hardware::CreateSerialGpsDriver(
                sensor.id, sensor.params);
        }
        if (sensor.backend == "can") {
            return autodriver::hardware::CreateCanGpsDriver(sensor.id,
                                                            sensor.params);
        }
        AERROR << "unknown GPS backend: " << sensor.backend;
        return nullptr;
    }
};

CLASS_LOADER_REGISTER_CLASS(GpsModule, autodriver::SensorModule)

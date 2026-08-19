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

#ifdef AUTODRIVER_HAVE_REALSENSE
#include "autodriver/hardware/realsense_camera_driver.hpp"
#endif
#include "autodriver/sensor_plugin.hpp"
#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"

class CameraModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kCamera> {
protected:
    std::shared_ptr<autodriver::SensorDriver> MakeDriver(
        const autodriver::Config::Sensor& sensor) override {
        if (sensor.backend != "realsense") {
            AERROR << "unknown camera backend: " << sensor.backend;
            return nullptr;
        }
#ifdef AUTODRIVER_HAVE_REALSENSE
        return autodriver::hardware::CreateRealSenseCameraDriver(
            sensor.id, sensor.params);
#else
        AERROR << "camera realsense backend not compiled";
        return nullptr;
#endif
    }
};

CLASS_LOADER_REGISTER_CLASS(CameraModule, autodriver::SensorModule)

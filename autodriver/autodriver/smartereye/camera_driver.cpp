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

#include "autodriver/smartereye/camera_driver.hpp"

#include "autodriver/camera/backend_register.hpp"
#include "autolink/common/log.hpp"

namespace autodriver {
namespace hardware {

std::shared_ptr<SensorDriver> CreateSmartereyeCameraDriver(
    const SensorId& id, const DriverParams& /*params*/) {
    AERROR << "Smartereye camera backend not implemented (id=" << id
           << "); link vendor SDK under smartereye/";
    return nullptr;
}

}  // namespace hardware
}  // namespace autodriver

REGISTER_CAMERA_BACKEND(smartereye_camera, "smartereye",
                        autodriver::hardware::CreateSmartereyeCameraDriver);

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

#ifndef AUTODRIVER_CONFIG_AUTODRIVER_CONFIG_HPP_
#define AUTODRIVER_CONFIG_AUTODRIVER_CONFIG_HPP_

#include <string>
#include <vector>

#include "autodriver/common/sensor_id.hpp"
#include "autodriver/drivers/hardware/driver_params.hpp"
#include "autodriver/sync/sensor_hub.hpp"

namespace autodriver {

struct DeviceMatch {
    std::string subsystem;
    std::string devnode;
    std::string vendor;
    std::string product;
    std::string serial;
};

struct SensorConfig {
    std::string class_name;
    std::string library;
    SensorId sensor_id;
    std::string channel;
    std::string backend;
    bool attach_on_start = false;
    DeviceMatch match;
    hardware::DriverParams params;
};

struct HotplugOptions {
    bool enable_udev = true;
};

struct AlignmentConfig {
    bool enable = false;
    SensorHubOptions hub_options;
};

struct AutodriverConfig {
    std::string node_name = "autodriver";
    std::string plugin_dir;
    HotplugOptions hotplug;
    AlignmentConfig alignment;
    std::vector<SensorConfig> sensors;
};

bool HasDuplicateSensorId(const AutodriverConfig& config);

}  // namespace autodriver

#endif  // AUTODRIVER_CONFIG_AUTODRIVER_CONFIG_HPP_

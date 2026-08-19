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

#ifndef AUTODRIVER_CONFIG_HPP_
#define AUTODRIVER_CONFIG_HPP_

#include <string>
#include <vector>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_hub.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {

// Observed or expected device identity (udev ADD/REMOVE, Sensor::match).
struct DeviceMatch {
    std::string subsystem;
    std::string device;
    std::string vendor;
    std::string product;
    std::string serial;

    bool empty() const {
        return subsystem.empty() && device.empty() && vendor.empty() &&
               product.empty() && serial.empty();
    }
};

bool MatchDevice(const DeviceMatch& observed, const DeviceMatch& rule);

// Process snapshot: N sensors. Autolink Node/Writers live in bridge.
struct Config {
    struct Sensor {
        std::string module;
        std::string library;
        SensorId id;
        std::vector<std::string> channels;
        std::string backend;
        bool autostart = false;
        DeviceMatch match;
        hardware::DriverParams params;
    };

    struct Hotplug {
        bool udev = true;
    };

    struct Alignment {
        bool enable = false;
        SensorHub::Options options;
    };

    std::string node_name = "autodriver";
    std::string plugins;
    Hotplug hotplug;
    Alignment alignment;
    std::vector<Sensor> sensors;

    bool HasDuplicateId() const;
    SensorId FindId(const DeviceMatch& observed) const;
};

}  // namespace autodriver

#endif  // AUTODRIVER_CONFIG_HPP_

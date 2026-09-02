/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Process configuration loaded from autodriver_hardware.yaml.
 */

#ifndef AUTODRIVER_CONFIG_HPP_
#define AUTODRIVER_CONFIG_HPP_

#include <string>
#include <vector>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_hub.hpp"
#include "autodriver/sensor_id.hpp"

namespace autodriver {

/**
 * @struct autodriver::DeviceMatch
 * @brief Observed or expected device identity (udev ADD/REMOVE, Sensor::match).
 */
struct DeviceMatch {
    // udev subsystem, e.g. "usb" or "iio".
    std::string subsystem;

    // udev DEVNAME or kernel device node.
    std::string device;

    // USB or PCI vendor id string.
    std::string vendor;

    // USB or PCI product id string.
    std::string product;

    // Device serial number when available.
    std::string serial;

    bool empty() const {
        return subsystem.empty() && device.empty() && vendor.empty() &&
               product.empty() && serial.empty();
    }
};

/**
 * @brief Returns true when all non-empty rule fields match the observed device.
 */
bool MatchDevice(const DeviceMatch& observed, const DeviceMatch& rule);

/**
 * @struct autodriver::Config
 * @brief In-memory snapshot of the autodriver process configuration.
 * Autolink Node/Writers live in bridge/; this struct only describes sensors.
 */
struct Config {
    /**
     * @brief One configured sensor instance.
     */
    struct Sensor {
        // class_loader plugin class name.
        std::string module;

        // Shared library path or basename for the plugin.
        std::string library;

        // Stable instance identifier, e.g. "imu/torso".
        SensorId id;

        // Explicit Autolink channel names; empty uses defaults.
        std::vector<std::string> channels;

        // Hardware backend name passed to the plugin factory.
        std::string backend;

        // Attach automatically when the process starts.
        bool autostart = false;

        // udev match rule for hotplug attach/detach.
        DeviceMatch match;

        // Driver-specific key/value parameters from YAML.
        hardware::DriverParams params;
    };

    /**
     * @brief Hotplug monitoring settings.
     */
    struct Hotplug {
        // Enable udev-based device arrival and removal handling.
        bool udev = true;
    };

    /**
     * @brief Multi-sensor time alignment settings.
     */
    struct Alignment {
        // Enable periodic aligned snapshot publishing.
        bool enable = false;

        // Hub options used when alignment is enabled.
        SensorHub::Options options;
    };

    // Autolink node name for bridge publishing.
    std::string node_name = "autodriver";

    // Directory or search path for sensor plugin libraries.
    std::string plugins;

    // Hotplug configuration block.
    Hotplug hotplug;

    // Alignment configuration block.
    Alignment alignment;

    // All configured sensor instances.
    std::vector<Sensor> sensors;

    bool HasDuplicateId() const;

    SensorId FindId(const DeviceMatch& observed) const;
};

}  // namespace autodriver

#endif  // AUTODRIVER_CONFIG_HPP_

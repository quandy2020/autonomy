/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file config.hpp
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

    /**
     * @brief Whether this match rule / observation has no identifying fields.
     * @return true when subsystem, device, vendor, product, and serial are all empty.
     */
    bool empty() const {
        return subsystem.empty() && device.empty() && vendor.empty() &&
               product.empty() && serial.empty();
    }
};

/**
 * @brief Returns true when all non-empty rule fields match the observed device.
 * @param[in] observed Device identity reported by udev or probe.
 * @param[in] rule Match rule from Config::Sensor::match (empty fields are wildcards).
 * @return true when every non-empty field in @p rule equals @p observed.
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
     *
     * When @c enable is true, every sample is tapped into SensorHub.
     * @c publish_raw controls whether the same sample still goes to SampleSink
     * (raw path). @c publish_aligned, when true, makes SensorManager publish
     * each sample in an AlignedSnapshot through SampleSink (aligned path).
     * Default matches historical behavior: raw on, aligned off (callback only).
     */
    struct Alignment {
        // Tap samples into SensorHub and run the alignment loop.
        bool enable = false;

        // When enable is true, also forward raw samples to SampleSink.
        bool publish_raw = true;

        // When enable is true, publish AlignedSnapshot samples via SampleSink.
        bool publish_aligned = false;

        // Hub options used when alignment is enabled.
        SensorHub::Options options;
    };

    /**
     * @brief Motion compensator pose feed (Odometry → PushLidarPose).
     */
    struct Compensator {
        // Autolink channel for nav_msgs/Odometry (empty = PoseFeeder disabled).
        std::string pose_channel;
    };

    /**
     * @brief Robot-body hardware (chassis) — separate from sensors.
     *
     * Lives in top-level chassis/; does not use autonomy/vehicle.
     * When enable=false, ChassisManager is a no-op.
     *
     * Abstraction knobs (locomotion / capability / mode / tool) are consumed
     * by ChassisManager; wire channels stay Twist / RobotState / String.
     */
    struct Chassis {
        /** @brief When false, ChassisManager::Start is a no-op. */
        bool enable = false;
        /** @brief Instance id, e.g. "chassis/base". */
        std::string id = "chassis/base";
        /** @brief ChassisBackendRegistry key (stub / scout / …). */
        std::string backend = "stub";
        /** @brief Autolink channel for TwistStamped commands. */
        std::string cmd_vel_channel = "/cmd_vel";
        /** @brief Autolink channel for vehicle_msgs.RobotState. */
        std::string state_channel = "/robot_state";
        /** @brief Autolink channel for RobotEvent; empty = do not publish. */
        std::string event_channel = "/robot_event";
        /** @brief Autolink channel for nav_msgs/Odometry; empty = skip. */
        std::string odom_channel = "/odom";
        /**
         * @brief Latched-style capability JSON (std_msgs/String); empty = skip.
         * Republished periodically with state for late subscribers.
         */
        std::string capability_channel = "/chassis/capability";
        /** @brief Mode commands (arm/estop/walk/…); empty = disable reader. */
        std::string mode_cmd_channel = "/chassis/mode";
        /** @brief Current mode string publisher; empty = skip. */
        std::string mode_state_channel = "/chassis/mode_state";
        /** @brief Tool commands (brush=1); empty = disable tool reader. */
        std::string tool_cmd_channel;
        /** @brief Soft-stop if no cmd_vel for this long; 0 disables watchdog. */
        int watchdog_ms = 200;
        /** @brief Soft clamp before ApplyVelocityCommand; 0 = no clamp. */
        double max_linear_speed = 0.0;
        /** @brief Soft clamp for angular.z; 0 = no clamp. */
        double max_angular_speed = 0.0;
        /** @brief Soft max linear accel (advertised; not yet rate-limited). */
        double max_linear_accel = 0.0;
        /** @brief Ackermann min turning radius (m); 0 = N/A. */
        double min_turning_radius = 0.0;
        /**
         * @brief Locomotion model name: differential / omni / ackermann /
         *        legged / wheel_legged / humanoid.
         */
        std::string locomotion = "differential";
        /** @brief Override supports_lateral; empty = derive from locomotion. */
        std::string supports_lateral;
        /** @brief Override supports_inplace_turn; empty = derive. */
        std::string supports_inplace_turn;
        /** @brief Must arm via mode channel before twist (SafetyGate). */
        bool require_arm = false;
        /** @brief Advertise dock capability. */
        bool has_dock = false;
        /** @brief Advertise joint bypass side-channel (docs only). */
        bool has_joint_bypass = false;
        /** @brief Tool names advertised in capability JSON. */
        std::vector<std::string> tools;
        /** @brief RobotState / odom publish period in milliseconds. */
        int odom_period_ms = 20;
        /** @brief Capability republish every N state ticks (0 = every tick). */
        int capability_period_ticks = 50;
        /** @brief Odometry header.frame_id. */
        std::string odom_frame_id = "odom";
        /** @brief Odometry child_frame_id. */
        std::string base_frame_id = "base_link";
        /** @brief Backend-specific key/value map passed to CreateDriver. */
        hardware::DriverParams params;
    };

    // Autolink node name for bridge publishing.
    std::string node_name = "autodriver";

    // Directory or search path for sensor plugin libraries.
    std::string plugins;

    // Hotplug configuration block.
    Hotplug hotplug;

    // Alignment configuration block.
    Alignment alignment;

    // Process-level compensator pose subscription.
    Compensator compensator;

    // Robot body / chassis hardware (optional).
    Chassis chassis;

    // All configured sensor instances.
    std::vector<Sensor> sensors;

    /**
     * @brief Returns true when two or more sensors share the same id.
     * @return true if config.sensors contains duplicate SensorId values.
     */
    bool HasDuplicateId() const;

    /**
     * @brief Returns the sensor id whose match rule fits the observed device.
     * @param[in] observed Device identity from a hotplug event.
     * @return Matching SensorId, or empty string when no rule matches.
     */
    SensorId FindId(const DeviceMatch& observed) const;
};

}  // namespace autodriver

#endif  // AUTODRIVER_CONFIG_HPP_

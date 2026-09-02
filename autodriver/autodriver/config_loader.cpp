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
 * @brief YAML parser for autodriver_hardware.yaml sensor entries.
 */

#include "autodriver/config_loader.hpp"

#include <cctype>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include "autodriver/common/environment.hpp"
#include "autodriver/conf/conf.hpp"
#include "autodriver/driver_params.hpp"
#include "autolink/common/file.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/duration.hpp"
#include <yaml-cpp/yaml.h>

namespace autodriver {
namespace {

using autolink::common::GetAbsolutePath;
using autolink::common::PathExists;

/**
 * @brief Converts a YAML scalar node to string, trying multiple types.
 */
std::string ScalarToString(const YAML::Node& node) {
    if (!node || node.IsNull() || !node.IsScalar()) {
        return {};
    }
    try {
        return node.as<std::string>();
    } catch (const YAML::Exception&) {
    }
    try {
        return std::to_string(node.as<int>());
    } catch (const YAML::Exception&) {
    }
    try {
        return std::to_string(node.as<double>());
    } catch (const YAML::Exception&) {
    }
    try {
        return node.as<bool>() ? "true" : "false";
    } catch (const YAML::Exception&) {
    }
    return {};
}

/**
 * @brief Reads a string field from a YAML map node.
 */
std::string ReadString(const YAML::Node& node, const char* key) {
    return ScalarToString(node[key]);
}

/**
 * @brief Reads an integer field, returning default_value when absent or invalid.
 */
int ReadInt(const YAML::Node& node, const char* key, int default_value = 0) {
    const YAML::Node value = node[key];
    if (!value || value.IsNull()) {
        return default_value;
    }
    if (value.IsScalar()) {
        try {
            return value.as<int>();
        } catch (const YAML::Exception&) {
            return default_value;
        }
    }
    return default_value;
}

/**
 * @brief Reads a boolean field, returning default_value when absent or invalid.
 */
bool ReadBool(const YAML::Node& node, const char* key, bool default_value) {
    const YAML::Node value = node[key];
    if (!value || value.IsNull()) {
        return default_value;
    }
    try {
        return value.as<bool>();
    } catch (const YAML::Exception&) {
        return default_value;
    }
}

/**
 * @brief Sets a string driver param only when key is unset and value is non-empty.
 */
void SetParamIfAbsent(hardware::DriverParams* params, const std::string& key,
                      const std::string& value) {
    if (value.empty() || params->count(key) != 0) {
        return;
    }
    (*params)[key] = value;
}

/**
 * @brief Sets an integer driver param only when key is unset and value is positive.
 */
void SetParamIfAbsent(hardware::DriverParams* params, const std::string& key,
                      int value) {
    if (value <= 0 || params->count(key) != 0) {
        return;
    }
    (*params)[key] = std::to_string(value);
}


/**
 * @brief Parses a DeviceMatch block from a YAML map node.
 */
DeviceMatch ReadMatch(const YAML::Node& node) {
    DeviceMatch out;
    if (!node || !node.IsMap()) {
        return out;
    }
    out.subsystem = ReadString(node, "subsystem");
    out.device = ReadString(node, "device");
    out.vendor = ReadString(node, "vendor");
    out.product = ReadString(node, "product");
    out.serial = ReadString(node, "serial");
    return out;
}

/**
 * @brief Copies all entries from a YAML params map into DriverParams.
 */
void ReadParamsMap(const YAML::Node& node, hardware::DriverParams* params) {
    if (!node || !node.IsMap() || params == nullptr) {
        return;
    }
    for (const auto& entry : node) {
        params->emplace(ScalarToString(entry.first), ScalarToString(entry.second));
    }
}

/**
 * @brief Derives udev match fields from serial device params when missing.
 */
void FinalizeSensor(Config::Sensor* sensor) {
    const auto device_it = sensor->params.find("device");
    if (device_it != sensor->params.end() && !device_it->second.empty() &&
        sensor->match.device.empty() &&
        (sensor->backend == "serial" || sensor->backend.empty())) {
        sensor->match.subsystem = "tty";
        sensor->match.device = device_it->second;
    }
}

/**
 * @brief Prefixes a bare sensor name with id_prefix unless already qualified.
 */
std::string QualifySensorId(const std::string& name, const char* prefix) {
    if (name.empty()) {
        return {};
    }
    if (name.find('/') != std::string::npos) {
        return name;
    }
    return std::string(prefix) + name;
}

/**
 * @brief Maps hardware shorthand YAML fields into backend-specific params.
 */
void ApplyHardwareShorthand(const YAML::Node& hardware,
                            const std::string& backend,
                            hardware::DriverParams* params) {
    if (!hardware) {
        return;
    }
    if (backend == "serial" || backend.empty()) {
        std::string port = ReadString(hardware, "port");
        if (port.empty()) {
            port = ReadString(hardware, "device");
        }
        SetParamIfAbsent(params, "device", port);
        int baud = ReadInt(hardware, "baudrate");
        if (baud <= 0) {
            baud = ReadInt(hardware, "baud");
        }
        SetParamIfAbsent(params, "baud", baud);
        return;
    }
    if (backend == "can") {
        SetParamIfAbsent(params, "interface", ReadString(hardware, "interface"));
        SetParamIfAbsent(params, "accel_can_id",
                        ReadString(hardware, "accel_can_id"));
        SetParamIfAbsent(params, "gyro_can_id",
                        ReadString(hardware, "gyro_can_id"));
        SetParamIfAbsent(params, "can_id", ReadString(hardware, "can_id"));
        SetParamIfAbsent(params, "accel_scale",
                        ReadString(hardware, "accel_scale"));
        SetParamIfAbsent(params, "gyro_scale", ReadString(hardware, "gyro_scale"));
        return;
    }
    if (backend == "realsense") {
        SetParamIfAbsent(params, "model", ReadString(hardware, "model"));
        SetParamIfAbsent(params, "stream", ReadString(hardware, "stream"));
        SetParamIfAbsent(params, "width", ReadInt(hardware, "width"));
        SetParamIfAbsent(params, "height", ReadInt(hardware, "height"));
        SetParamIfAbsent(params, "fps", ReadInt(hardware, "fps"));
    }
}

/**
 * @brief Reads publish channel names from scalar or sequence YAML nodes.
 */
void ReadChannels(const YAML::Node& node,
                  std::vector<std::string>* channels) {
    if (!node || !node["channel"] || channels == nullptr) {
        return;
    }
    const YAML::Node ch = node["channel"];
    if (ch.IsSequence()) {
        for (const auto& item : ch) {
            const std::string value = ScalarToString(item);
            if (!value.empty()) {
                channels->push_back(value);
            }
        }
        return;
    }
    const std::string value = ScalarToString(ch);
    if (!value.empty()) {
        channels->push_back(value);
    }
}

/**
 * @brief Applies publisher shorthand for channels and publish rate.
 */
void ApplyPublisherShorthand(const YAML::Node& publisher,
                             std::vector<std::string>* channels,
                             hardware::DriverParams* params) {
    if (!publisher) {
        return;
    }
    if (channels != nullptr && channels->empty()) {
        ReadChannels(publisher, channels);
    }
    int rate = ReadInt(publisher, "rate");
    if (rate <= 0) {
        rate = ReadInt(publisher, "fps");
    }
    SetParamIfAbsent(params, "publish_rate_hz", rate);
}

/**
 * @brief Merges flat device YAML fields into params and channel lists.
 */
void ApplyFlatDeviceFields(const YAML::Node& node, const std::string& backend,
                           const std::string& module,
                           hardware::DriverParams* params,
                           std::vector<std::string>* channels) {
    ApplyHardwareShorthand(node, backend, params);
    if (node["hardware"]) {
        ApplyHardwareShorthand(node["hardware"], backend, params);
    }

    ReadChannels(node, channels);
    ApplyPublisherShorthand(node["publisher"], channels, params);

    int fps = ReadInt(node, "fps");
    if (module == "CameraModule" || module == "PointCloudModule") {
        SetParamIfAbsent(params, "fps", fps);
        SetParamIfAbsent(params, "width", ReadInt(node, "width"));
        SetParamIfAbsent(params, "height", ReadInt(node, "height"));
        SetParamIfAbsent(params, "model", ReadString(node, "model"));
        SetParamIfAbsent(params, "stream", ReadString(node, "stream"));
        SetParamIfAbsent(params, "frame_id", ReadString(node, "frame_id"));
    } else {
        int rate = ReadInt(node, "rate");
        if (rate <= 0) {
            rate = fps;
        }
        SetParamIfAbsent(params, "publish_rate_hz", rate);
    }
}

/**
 * @brief Module name, id prefix, and default backend for a sensor group.
 */
struct DeviceKind {
    const char* module;
    const char* id_prefix;
    const char* default_backend;
};

/**
 * @brief Returns true when enable or legacy attach_on_start is set.
 */
bool IsDeviceEnabled(const YAML::Node& node) {
    if (ReadBool(node, "enable", false)) {
        return true;
    }
    // Legacy alias.
    return ReadBool(node, "attach_on_start", false);
}

/**
 * @brief Builds a typed Config::Sensor from a YAML device entry.
 */
Config::Sensor TypedDeviceFromYaml(const YAML::Node& node,
                                   const DeviceKind& kind) {
    Config::Sensor out;
    out.module = kind.module;
    out.backend = ReadString(node, "backend");
    if (out.backend.empty()) {
        out.backend = kind.default_backend;
    }
    out.id = QualifySensorId(ReadString(node, "name"), kind.id_prefix);
    out.autostart = true;
    out.match = ReadMatch(node["match"]);
    ReadParamsMap(node["params"], &out.params);
    ApplyFlatDeviceFields(node, out.backend, out.module, &out.params,
                          &out.channels);
    FinalizeSensor(&out);
    return out;
}

/**
 * @brief Appends enabled devices from a YAML sequence using kind metadata.
 */
void AppendTypedList(const YAML::Node& devices, const DeviceKind& kind,
                     std::vector<Config::Sensor>* sensors) {
    if (!devices || !devices.IsSequence()) {
        return;
    }
    for (const auto& device : devices) {
        if (!IsDeviceEnabled(device)) {
            continue;
        }
        sensors->push_back(TypedDeviceFromYaml(device, kind));
    }
}

/**
 * @brief Returns true when dimension/type indicates a 3D lidar device.
 */
bool IsLidar3d(const YAML::Node& node) {
    std::string dim = ReadString(node, "dimension");
    if (dim.empty()) {
        dim = ReadString(node, "type");
    }
    for (char& c : dim) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return dim == "3d" || dim == "lidar3d" || dim == "pointcloud";
}

/**
 * @brief Appends lidar devices, choosing 2D or 3D module by dimension field.
 */
void AppendLidarList(const YAML::Node& devices,
                     std::vector<Config::Sensor>* sensors) {
    if (!devices || !devices.IsSequence()) {
        return;
    }
    // Default metadata for 2D lidar entries in mixed lidar lists.
    static constexpr DeviceKind kLidar2d{"Lidar2dModule", "lidar/", "serial"};

    // Default metadata for 3D lidar entries in mixed lidar lists.
    static constexpr DeviceKind kLidar3d{"Lidar3dModule", "lidar/", "serial"};
    for (const auto& device : devices) {
        if (!IsDeviceEnabled(device)) {
            continue;
        }
        const DeviceKind& kind = IsLidar3d(device) ? kLidar3d : kLidar2d;
        sensors->push_back(TypedDeviceFromYaml(device, kind));
    }
}

/**
 * @brief Appends all known sensor groups from a YAML map.
 */
void AppendSensorGroups(const YAML::Node& groups,
                        std::vector<Config::Sensor>* sensors) {
    static constexpr DeviceKind kImu{"ImuModule", "imu/", "serial"};
    static constexpr DeviceKind kGps{"GpsModule", "gps/", "serial"};
    static constexpr DeviceKind kLidar2d{"Lidar2dModule", "lidar/", "serial"};
    static constexpr DeviceKind kLidar3d{"Lidar3dModule", "lidar/", "serial"};
    static constexpr DeviceKind kCamera{"CameraModule", "camera/", "realsense"};
    static constexpr DeviceKind kRange{"RangeModule", "range/", "serial"};
    static constexpr DeviceKind kPointCloud{"PointCloudModule", "camera/",
                                            "realsense"};

    AppendTypedList(groups["lidar_2d"], kLidar2d, sensors);
    AppendTypedList(groups["lidar_3d"], kLidar3d, sensors);
    AppendLidarList(groups["lidar"], sensors);
    AppendTypedList(groups["point_cloud"], kPointCloud, sensors);
    AppendTypedList(groups["imu"], kImu, sensors);
    AppendTypedList(groups["imu_devices"], kImu, sensors);
    AppendTypedList(groups["gps"], kGps, sensors);
    AppendTypedList(groups["gps_devices"], kGps, sensors);
    AppendTypedList(groups["camera"], kCamera, sensors);
    AppendTypedList(groups["range"], kRange, sensors);
}

/**
 * @brief Parses a legacy flat sensor entry from YAML.
 */
Config::Sensor SensorFromYaml(const YAML::Node& node) {
    if (!IsDeviceEnabled(node)) {
        return {};
    }
    Config::Sensor out;
    out.module = ReadString(node, "module");
    out.library = ReadString(node, "library");
    out.id = ReadString(node, "sensor_id");
    out.backend = ReadString(node, "backend");
    out.autostart = true;
    ReadChannels(node, &out.channels);
    out.match = ReadMatch(node["match"]);
    ReadParamsMap(node["params"], &out.params);

    SetParamIfAbsent(&out.params, "device", ReadString(node, "device"));
    SetParamIfAbsent(&out.params, "baud", ReadInt(node, "baud"));
    SetParamIfAbsent(&out.params, "interface", ReadString(node, "interface"));
    SetParamIfAbsent(&out.params, "model", ReadString(node, "model"));
    SetParamIfAbsent(&out.params, "stream", ReadString(node, "stream"));
    SetParamIfAbsent(&out.params, "frame_id", ReadString(node, "frame_id"));
    SetParamIfAbsent(&out.params, "width", ReadInt(node, "width"));
    SetParamIfAbsent(&out.params, "height", ReadInt(node, "height"));
    SetParamIfAbsent(&out.params, "fps", ReadInt(node, "fps"));
    SetParamIfAbsent(&out.params, "accel_can_id",
                    ReadString(node, "accel_can_id"));
    SetParamIfAbsent(&out.params, "gyro_can_id",
                    ReadString(node, "gyro_can_id"));
    SetParamIfAbsent(&out.params, "can_id", ReadString(node, "can_id"));
    SetParamIfAbsent(&out.params, "accel_scale",
                    ReadString(node, "accel_scale"));
    SetParamIfAbsent(&out.params, "gyro_scale", ReadString(node, "gyro_scale"));

    FinalizeSensor(&out);
    return out;
}

/**
 * @brief Builds a Config from the root YAML document.
 */
Config FromYaml(const YAML::Node& root) {
    Config config;
    const std::string node_name = ReadString(root, "node_name");
    if (!node_name.empty()) {
        config.node_name = node_name;
    }
    config.plugins = ReadString(root, "plugin_dir");

    if (root["hotplug"]) {
        config.hotplug.udev =
            ReadBool(root["hotplug"], "enable_udev", config.hotplug.udev);
    }

    if (root["alignment"]) {
        const YAML::Node alignment = root["alignment"];
        config.alignment.enable =
            ReadBool(alignment, "enable", config.alignment.enable);
        const int alignment_window_ms =
            ReadInt(alignment, "alignment_window_ms");
        if (alignment_window_ms > 0) {
            config.alignment.options.alignment_window = autolink::Duration(
                static_cast<int64_t>(alignment_window_ms) * 1'000'000);
        }
        const int publish_period_ms = ReadInt(alignment, "publish_period_ms");
        if (publish_period_ms > 0) {
            config.alignment.options.publish_period = autolink::Duration(
                static_cast<int64_t>(publish_period_ms) * 1'000'000);
        }
        const int buffer_capacity = ReadInt(alignment, "buffer_capacity");
        if (buffer_capacity > 0) {
            config.alignment.options.buffer_capacity =
                static_cast<std::size_t>(buffer_capacity);
        }
    }

    const YAML::Node sensors_node = root["sensors"];
    if (sensors_node && sensors_node.IsMap()) {
        AppendSensorGroups(sensors_node, &config.sensors);
    } else {
        AppendSensorGroups(root, &config.sensors);
    }
    if (sensors_node && sensors_node.IsSequence()) {
        config.sensors.reserve(config.sensors.size() +
                               static_cast<std::size_t>(sensors_node.size()));
        for (const auto& sensor_node : sensors_node) {
            Config::Sensor sensor = SensorFromYaml(sensor_node);
            if (sensor.id.empty()) {
                continue;
            }
            config.sensors.push_back(std::move(sensor));
        }
    }
    if (config.sensors.empty()) {
        AINFO << "autodriver config: no enabled sensors";
    }
    return config;
}

/**
 * @brief Resolves an autodriver config path from work root or install tree.
 */
std::string ResolveConfigPath(const std::string& configuration_directory,
                              const std::string& config_basename) {
    /**
     * @brief Config filename, defaulting to kDefaultConfigBasename when empty.
     */
    std::string basename = config_basename.empty() ? kDefaultConfigBasename
                                                   : config_basename;
    if (!basename.empty() && basename.front() == '/') {
        return basename;
    }

    /**
     * @brief Work root used to locate config/ when no directory is given.
     */
    const std::string root = configuration_directory.empty()
                                 ? common::WorkRoot()
                                 : configuration_directory;
    const std::string path = GetAbsolutePath(root, "config/" + basename);
    if (PathExists(path)) {
        return path;
    }

    /**
     * @brief Install-tree fallback when source-tree config is missing.
     */
    const std::string install_path = GetAbsolutePath(
        conf::kDefaultDistributionHome, "share/autodriver/config/" + basename);
    if (PathExists(install_path)) {
        AINFO << "config_path: " << install_path << " (install-tree fallback)";
        return install_path;
    }

    AINFO << "config_path: " << path;
    return path;
}

/**
 * @brief Loads and parses a YAML config file at path.
 */
Config LoadFromPath(const std::string& path) {
    try {
        return FromYaml(YAML::LoadFile(path));
    } catch (const YAML::Exception& ex) {
        throw std::runtime_error("failed to load autodriver config: " + path +
                                 " (" + ex.what() +
                                 "; optional override: export "
                                 "AUTODRIVER_PATH=<config parent>)");
    }
}

}  // namespace

/**
 * @brief Loads the default autodriver_hardware.yaml config.
 */
Config LoadConfig() { return LoadConfig(kDefaultConfigBasename); }

/**
 * @brief Loads config by basename from the default configuration directory.
 */
Config LoadConfig(const std::string& config_basename) {
    return LoadConfig({}, config_basename);
}

/**
 * @brief Loads config from an explicit directory and basename.
 */
Config LoadConfig(const std::string& configuration_directory,
                  const std::string& config_basename) {
    return LoadFromPath(
        ResolveConfigPath(configuration_directory, config_basename));
}

}  // namespace autodriver

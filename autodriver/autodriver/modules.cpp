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
 * @brief Registers all built-in sensor modules with the class_loader so that
 * SensorManager can instantiate them without loading a separate .so.
 */

#include "autodriver/sensor_plugin.hpp"
#include "autodriver/imu/can_imu_driver.hpp"
#include "autodriver/imu/serial_imu_driver.hpp"
#include "autodriver/gps/can_gps_driver.hpp"
#include "autodriver/gps/serial_gps_driver.hpp"
#include "autodriver/camera/backend_registry.hpp"
#include "autodriver/lidar/backend_registry.hpp"
#include "autodriver/radar/backend_registry.hpp"
#include "autodriver/microphone/backend_registry.hpp"
#ifdef AUTODRIVER_HAVE_REALSENSE
#include "autodriver/camera/realsense/imu_driver.hpp"
#endif

#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"

// ---------------------------------------------------------------------------
// IMU module — serial, CAN, and RealSense backends.
// ---------------------------------------------------------------------------
/**
 * @class ImuModule
 * @brief SensorPlugin for IMU devices (serial WIT, CAN, RealSense).
 */
class ImuModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kImu> {
protected:
    /**
     * @brief Instantiates a serial, CAN, or RealSense IMU driver from config.
     */
    std::shared_ptr<autodriver::SensorDriver> MakeDriver(
        const autodriver::Config::Sensor& sensor) override {
        if (sensor.backend == "serial") {
            return autodriver::hardware::CreateSerialImuDriver(
                sensor.id, sensor.params);
        }
        if (sensor.backend == "can") {
            return autodriver::hardware::CreateCanImuDriver(
                sensor.id, sensor.params);
        }
        if (sensor.backend == "realsense") {
#ifdef AUTODRIVER_HAVE_REALSENSE
            return autodriver::hardware::CreateRealSenseImuDriver(
                sensor.id, sensor.params);
#else
            AERROR << "IMU realsense backend not compiled";
            return nullptr;
#endif
        }
        AERROR << "unknown IMU backend: " << sensor.backend;
        return nullptr;
    }
};

CLASS_LOADER_REGISTER_CLASS(ImuModule, autodriver::SensorModule)

// ---------------------------------------------------------------------------
// GPS module — serial and CAN backends.
// ---------------------------------------------------------------------------
/**
 * @class GpsModule
 * @brief SensorPlugin for GNSS receivers (serial NMEA, CAN NMEA2000).
 */
class GpsModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kGps> {
protected:
    /**
     * @brief Instantiates a serial or CAN GPS driver from config.
     */
    std::shared_ptr<autodriver::SensorDriver> MakeDriver(
        const autodriver::Config::Sensor& sensor) override {
        if (sensor.backend == "serial") {
            return autodriver::hardware::CreateSerialGpsDriver(
                sensor.id, sensor.params);
        }
        if (sensor.backend == "can") {
            return autodriver::hardware::CreateCanGpsDriver(
                sensor.id, sensor.params);
        }
        AERROR << "unknown GPS backend: " << sensor.backend;
        return nullptr;
    }
};

CLASS_LOADER_REGISTER_CLASS(GpsModule, autodriver::SensorModule)

// ---------------------------------------------------------------------------
// Camera module — backends via CameraBackendRegistry (realsense/orbbec/…).
// ---------------------------------------------------------------------------
/**
 * @class CameraModule
 * @brief SensorPlugin for RGB/depth/IR; backends via CameraBackendRegistry.
 */
class CameraModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kCamera> {
protected:
    /**
     * @brief Instantiates an Image driver from the camera backend registry.
     */
    std::shared_ptr<autodriver::SensorDriver> MakeDriver(
        const autodriver::Config::Sensor& sensor) override {
        return autodriver::camera::CameraBackendRegistry::Instance().Create(
            sensor.backend, sensor.id, sensor.params);
    }
};

CLASS_LOADER_REGISTER_CLASS(CameraModule, autodriver::SensorModule)

// ---------------------------------------------------------------------------
// Lidar modules — 2D/3D placeholders and RealSense point cloud.
// Vendors should use autodriver/lidar/lidar_component_base.hpp
// (Scan → Convert → PointCloud, SourceType online|raw_packet).
// ---------------------------------------------------------------------------
/**
 * @class Lidar2dModule
 * @brief Attach-only placeholder until a Stream/UDP backend lands.
 */
class Lidar2dModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kLidar2d,
                                      false> {};

/**
 * @class Lidar3dModule
 * @brief 3D lidar: backends via LidarBackendRegistry (e.g. velodyne|udp).
 */
class Lidar3dModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kLidar3d> {
protected:
    /**
     * @brief Instantiates a 3D lidar driver from LidarBackendRegistry.
     */
    std::shared_ptr<autodriver::SensorDriver> MakeDriver(
        const autodriver::Config::Sensor& sensor) override {
        return autodriver::lidar::LidarBackendRegistry::Instance().Create(
            sensor.backend, sensor.id, sensor.params);
    }
};
/**
 * @class PointCloudModule
 * @brief Depth/color point clouds; backends via PointCloudBackendRegistry.
 */
class PointCloudModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kLidar3d> {
protected:
    /**
     * @brief Instantiates a PointCloud2 driver from PointCloudBackendRegistry.
     */
    std::shared_ptr<autodriver::SensorDriver> MakeDriver(
        const autodriver::Config::Sensor& sensor) override {
        return autodriver::camera::PointCloudBackendRegistry::Instance().Create(
            sensor.backend, sensor.id, sensor.params);
    }
};

/**
 * @class LidarModule
 * @brief Alias module registered for legacy lidar plugin names.
 */
class LidarModule : public Lidar2dModule {};

CLASS_LOADER_REGISTER_CLASS(Lidar2dModule, autodriver::SensorModule)
CLASS_LOADER_REGISTER_CLASS(Lidar3dModule, autodriver::SensorModule)
CLASS_LOADER_REGISTER_CLASS(PointCloudModule, autodriver::SensorModule)
CLASS_LOADER_REGISTER_CLASS(LidarModule, autodriver::SensorModule)

// ---------------------------------------------------------------------------
// Range finder module — attach-only placeholder.
// ---------------------------------------------------------------------------
/**
 * @class RangeModule
 * @brief Attach-only placeholder for range finder plugins.
 */
class RangeModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kRangeFinder,
                                      false> {};

CLASS_LOADER_REGISTER_CLASS(RangeModule, autodriver::SensorModule)

// ---------------------------------------------------------------------------
// Radar / Microphone modules — backends via registries (stubs until wired).
// ---------------------------------------------------------------------------
/**
 * @class RadarModule
 * @brief SensorPlugin for automotive radar; backends via RadarBackendRegistry.
 */
class RadarModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kRadar> {
protected:
    /**
     * @brief Instantiates a radar driver (may be nullptr for stubs).
     */
    std::shared_ptr<autodriver::SensorDriver> MakeDriver(
        const autodriver::Config::Sensor& sensor) override {
        return autodriver::radar::RadarBackendRegistry::Instance().Create(
            sensor.backend, sensor.id, sensor.params);
    }
};

/**
 * @class MicrophoneModule
 * @brief SensorPlugin for audio capture; backends via MicrophoneBackendRegistry.
 */
class MicrophoneModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kMicrophone> {
protected:
    /**
     * @brief Instantiates a microphone driver (may be nullptr for stubs).
     */
    std::shared_ptr<autodriver::SensorDriver> MakeDriver(
        const autodriver::Config::Sensor& sensor) override {
        return autodriver::microphone::MicrophoneBackendRegistry::Instance()
            .Create(sensor.backend, sensor.id, sensor.params);
    }
};

CLASS_LOADER_REGISTER_CLASS(RadarModule, autodriver::SensorModule)
CLASS_LOADER_REGISTER_CLASS(MicrophoneModule, autodriver::SensorModule)

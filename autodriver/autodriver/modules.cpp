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
 *        SensorManager can instantiate them without loading a separate .so.
 */

#include "autodriver/sensor_plugin.hpp"
#include "autodriver/hardware/can_imu_driver.hpp"
#include "autodriver/hardware/serial_imu_driver.hpp"
#include "autodriver/hardware/can_gps_driver.hpp"
#include "autodriver/hardware/serial_gps_driver.hpp"
#ifdef AUTODRIVER_HAVE_REALSENSE
#include "autodriver/hardware/realsense_imu_driver.hpp"
#include "autodriver/hardware/realsense_camera_driver.hpp"
#include "autodriver/hardware/realsense_pointcloud_driver.hpp"
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
    /** @brief Instantiates a serial, CAN, or RealSense IMU driver from config. */
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
    /** @brief Instantiates a serial or CAN GPS driver from config. */
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
// Camera module — RealSense video streams.
// ---------------------------------------------------------------------------

/**
 * @class CameraModule
 * @brief SensorPlugin for RealSense color/depth/IR video streams.
 */
class CameraModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kCamera> {
protected:
    /** @brief Instantiates a RealSense camera driver from config. */
    std::shared_ptr<autodriver::SensorDriver> MakeDriver(
        const autodriver::Config::Sensor& sensor) override {
        if (sensor.backend != "realsense") {
            AERROR << "unsupported camera backend: " << sensor.backend;
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

// ---------------------------------------------------------------------------
// Lidar modules — 2D/3D placeholders and RealSense point cloud.
// ---------------------------------------------------------------------------

/** @class Lidar2dModule @brief Attach-only placeholder for 2D lidar plugins. */
class Lidar2dModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kLidar2d,
                                      false> {};

/** @class Lidar3dModule @brief Attach-only placeholder for 3D lidar plugins. */
class Lidar3dModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kLidar3d,
                                      false> {};

/**
 * @class PointCloudModule
 * @brief SensorPlugin for RealSense colored point clouds.
 */
class PointCloudModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kLidar3d> {
protected:
    /** @brief Instantiates a RealSense point-cloud driver from config. */
    std::shared_ptr<autodriver::SensorDriver> MakeDriver(
        const autodriver::Config::Sensor& sensor) override {
        if (sensor.backend != "realsense") {
            AERROR << "unsupported point cloud backend: " << sensor.backend;
            return nullptr;
        }
#ifdef AUTODRIVER_HAVE_REALSENSE
        return autodriver::hardware::CreateRealSensePointCloudDriver(
            sensor.id, sensor.params);
#else
        AERROR << "point cloud realsense backend not compiled";
        return nullptr;
#endif
    }
};

/** @class LidarModule @brief Alias module registered for legacy lidar plugin names. */
class LidarModule : public Lidar2dModule {};

CLASS_LOADER_REGISTER_CLASS(Lidar2dModule, autodriver::SensorModule)
CLASS_LOADER_REGISTER_CLASS(Lidar3dModule, autodriver::SensorModule)
CLASS_LOADER_REGISTER_CLASS(PointCloudModule, autodriver::SensorModule)
CLASS_LOADER_REGISTER_CLASS(LidarModule, autodriver::SensorModule)

// ---------------------------------------------------------------------------
// Range finder module — attach-only placeholder.
// ---------------------------------------------------------------------------

/** @class RangeModule @brief Attach-only placeholder for range finder plugins. */
class RangeModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kRangeFinder,
                                      false> {};

CLASS_LOADER_REGISTER_CLASS(RangeModule, autodriver::SensorModule)

/*
 * Copyright 2026 The Openbot Authors
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

/**
 * @file odometry_base.hpp
 * @brief Abstract LIO / LIVO odometry interface.
 */

#ifndef ARGUS_ODOMETRY_ODOMETRY_BASE_HPP_
#define ARGUS_ODOMETRY_ODOMETRY_BASE_HPP_

#include "argus/frontend/types.hpp"
#include "argus/sensor/imu_data.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace odometry {

/**
 * @brief Process outcome for host observability (still may return true).
 */
enum class ProcessStatus {
    kOk = 0,     /**< @brief Normal update completed. */
    kNoPackage,  /**< @brief Sync not ready (`Process` returns false). */
    kEmptyLidar, /**< @brief LO/LIO package with no points after undistort. */
    kAllDynamicRejected, /**< @brief Dynamic filter removed every point. */
    kVioNotReady,        /**< @brief Camera / VIOManager not initialized. */
    kEmptyImage,         /**< @brief VIO package without image. */
    kPropagateOnly,      /**< @brief WAIT / unknown flag; IMU propagate only. */
};

/**
 * @brief Stable string name for logging and metrics.
 * @param s Status value to stringify.
 * @return Lowercase snake_case label, or `"unknown"`.
 */
inline const char* ProcessStatusName(ProcessStatus s) {
    switch (s) {
        case ProcessStatus::kOk:
            return "ok";
        case ProcessStatus::kNoPackage:
            return "no_package";
        case ProcessStatus::kEmptyLidar:
            return "empty_lidar";
        case ProcessStatus::kAllDynamicRejected:
            return "all_dynamic_rejected";
        case ProcessStatus::kVioNotReady:
            return "vio_not_ready";
        case ProcessStatus::kEmptyImage:
            return "empty_image";
        case ProcessStatus::kPropagateOnly:
            return "propagate_only";
        default:
            return "unknown";
    }
}

/** @brief One odometry cycle output (state, clouds, status). */
struct OdometryOutput {
    frontend::StatesGroup state; /**< @brief Filter state after this cycle. */
    frontend::EkfState ekf_phase = frontend::kWait; /**< @brief EKF phase flag. */
    double timestamp = 0.0;                   /**< @brief Output stamp (s). */
    sensor::PointCloudXYZI::Ptr
        undistorted; /**< @brief Motion-compensated scan. */
    sensor::PointCloudXYZI::Ptr
        downsampled_body; /**< @brief Body-frame downsample for map. */
    bool map_updated =
        false; /**< @brief True when voxel / visual map changed. */
    ProcessStatus status =
        ProcessStatus::kOk; /**< @brief Outcome for observability. */
    /** @brief True when status != kOk but package was still consumed. */
    bool soft_skip = false;
};

/**
 * @brief Abstract odometry backend (LiDAR + IMU, optional vision).
 */
class OdometryBase
{
public:
    virtual ~OdometryBase() = default;

    /**
     * @brief Queue a LiDAR scan for synchronization.
     * @param cloud Point cloud in sensor frame.
     * @param header_time Scan header time (s).
     */
    virtual void AddLidar(const sensor::PointCloudXYZI::Ptr& cloud,
                          double header_time) = 0;

    /**
     * @brief Queue an IMU sample for propagation and undistortion.
     * @param imu IMU measurement.
     */
    virtual void AddImu(const sensor::ImuData& imu) = 0;

    /**
     * @brief Run one synced package when ready.
     * @param[out] out Filled on success or soft skip; may be partial on hard
     * failure.
     * @return False when no package is available (`kNoPackage`); true
     * otherwise.
     */
    virtual bool Process(OdometryOutput* out) = 0;

    /**
     * @brief Current filter state (read-only).
     * @return Reference to internal `StatesGroup`.
     */
    virtual const frontend::StatesGroup& state() const = 0;
};

}  // namespace odometry
}  // namespace argus

#endif  // ARGUS_ODOMETRY_ODOMETRY_BASE_HPP_

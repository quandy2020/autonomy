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
 * @file livo_odometry.hpp
 * @brief FAST-LIVO2 LIO orchestrator (sync → IMU undistort → voxel ESIKF).
 */

#ifndef ARGUS_ODOMETRY_LIVO_ODOMETRY_HPP_
#define ARGUS_ODOMETRY_LIVO_ODOMETRY_HPP_

#include <memory>
#include <unordered_map>

#include <opencv2/core.hpp>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "argus/common/camera/abstract_camera.hpp"
#include "argus/common/macros.hpp"
#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/frontend/imu_process.hpp"
#include "argus/frontend/preprocess.hpp"
#include "argus/frontend/sensor_sync.hpp"
#include "argus/frontend/types.hpp"
#include "argus/fusion/confidence_gating.hpp"
#include "argus/fusion/intensity_assist.hpp"
#include "argus/loop_closure/loop_closure_manager.hpp"
#include "argus/map/map_self_cleaning.hpp"
#include "argus/map/lio_frame_buffers.hpp"
#include "argus/map/voxel_map.hpp"
#include "argus/mapping/map_builder.hpp"
#include "argus/odometry/livo_odometry_config.hpp"
#include "argus/odometry/odometry_base.hpp"
#include "argus/odometry/vio_manager.hpp"
#include "argus/perception/degeneracy_detector.hpp"
#include "argus/perception/sensor_health_monitor.hpp"
#include "argus/relocalization/relocalization_manager.hpp"
#include "argus/sensor/image_data.hpp"
#include "argus/sensor/internal/voxel_grid.hpp"
#include "argus/sensor/point_cloud.hpp"

namespace argus {
namespace backend {
struct EsikfUpdateOptions;
}  // namespace backend

namespace odometry {

/** @brief FAST-LIVO2-style LIO/LIVO orchestrator with optional loop closure and
 * mapping. */
class LivoOdometry : public OdometryBase
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(LivoOdometry);

    /** @brief Default-constructs with internal default LivoOdometryConfig. */
    LivoOdometry();
    /**
     * @brief Construct with full configuration (Initialize still required).
     * @param config Odometry, VIO, loop closure, and mapping options.
     */
    explicit LivoOdometry(const LivoOdometryConfig& config);

    /**
     * @brief Replace runtime configuration (re-applies submodules on
     * Initialize).
     * @param config New configuration POD.
     */
    void SetConfig(const LivoOdometryConfig& config);
    /** @brief Wire preprocess, IMU, voxel map, VIO, and optional async workers.
     */
    void Initialize();

    /**
     * @brief Enqueue a LiDAR scan for sync and undistortion.
     * @param cloud Point cloud in sensor frame.
     * @param header_time Scan header time (s).
     */
    void AddLidar(const sensor::PointCloudXYZI::Ptr& cloud,
                  double header_time) override;
    /**
     * @brief Enqueue LiDAR from protobuf PointCloud2.
     * @param msg Serialized cloud message.
     * @param header_time Stamp override or message time (s).
     */
    void AddLidar(const ::automsgs::msgs::sensor_msgs::PointCloud2& msg,
                  double header_time);
    /**
     * @brief Enqueue IMU sample for propagation and undistortion.
     * @param imu IMU measurement.
     */
    void AddImu(const sensor::ImuData& imu) override;
    /**
     * @brief Enqueue camera frame for VIO when `enable_image`.
     * @param image Image payload and timestamp.
     */
    void AddImage(const sensor::ImageData& image);

    /**
     * @brief Process one synced package (LIO/LO → ESIKF; VIO → VIOManager).
     * @param[out] out Filled when a package is consumed (even on partial
     * failure).
     * @return false when sync has no ready package.
     */
    bool Process(OdometryOutput* out) override;

    /** @return Const reference to the ESIKF state after the last update. */
    const frontend::StatesGroup& state() const override {
        return state_;
    }
    /** @return Mutable ESIKF state (estimate thread only). */
    frontend::StatesGroup& mutable_state() {
        return state_;
    }
    /** @return IMU-propagated state used during scan undistortion. */
    frontend::StatesGroup& state_propagated() {
        return state_propagated_;
    }

    /** @return LiDAR preprocess (deskew / feature prep). */
    frontend::Preprocess& preprocess() {
        return *preprocess_;
    }
    /** @return Shared preprocess instance. */
    frontend::Preprocess::SharedPtr preprocess_ptr() {
        return preprocess_;
    }
    /** @return IMU process (init, propagate, undistort). */
    frontend::ImuProcess& imu_process() {
        return *imu_process_;
    }
    /** @return Shared IMU process instance. */
    frontend::ImuProcess::SharedPtr imu_ptr() {
        return imu_process_;
    }
    /** @return Sensor sync buffer and measure assembly. */
    frontend::SensorSync& sync() {
        return sync_;
    }
    /** @return Voxel map manager for ESIKF map updates. */
    map::VoxelMapManager& voxel_map() {
        return *voxel_manager_;
    }
    /** @return Shared voxel map manager. */
    map::VoxelMapManager::SharedPtr voxel_map_ptr() {
        return voxel_manager_;
    }

    /**
     * @brief Shared VIOManager (created when `enable_image`).
     *
     * Atlas may configure camera/extrinsics then call `InitializeVIO()` on it.
     * Argus alone loads from `config.vio` in `Initialize()`.
     */
    /** @return Shared VIOManager (null when `enable_image` is false). */
    VIOManager::SharedPtr vio_manager_ptr() {
        return vio_manager_;
    }
    /** @return Raw VIOManager pointer (may be null). */
    VIOManager* vio_manager() {
        return vio_manager_.get();
    }
    /** @return Const VIOManager pointer (may be null). */
    const VIOManager* vio_manager() const {
        return vio_manager_.get();
    }

    /**
     * @brief Create empty VIOManager if missing (Atlas wiring).
     * @return Shared VIOManager (existing or newly created).
     */
    VIOManager::SharedPtr EnsureVioManager();

    /**
     * @brief Load camera from `config.vio` and call `InitializeVIO()`.
     * @return true when VIO is ready for `Process` visual updates.
     */
    bool InitializeVio();

    /** @return True after successful `InitializeVIO` (width > 0). */
    bool vio_ready() const;
    /** @return Root voxel hash table backing the local map. */
    std::unordered_map<map::VoxelLocation, map::VoxelOctoTree*>& voxel_hash() {
        return voxel_hash_;
    }

    /** @return Current LiDAR measure group under sync. */
    frontend::LidarMeasureGroup& measures() {
        return measures_;
    }
    /** @return True after the first voxel map initialization pass. */
    bool lidar_map_inited() const {
        return lidar_map_inited_;
    }
    /** @return Timestamp (s) of the first received LiDAR scan. */
    double first_lidar_time() const {
        return first_lidar_time_;
    }

    /**
     * @brief Last LiDAR-update degeneracy result (valid after a LIO Process).
     * @return FIM-based degeneracy metrics from the last LIO cycle.
     */
    const perception::DegeneracyResult& last_degeneracy() const {
        return last_degeneracy_;
    }

    /** @return Fraction of downsampled LiDAR points rejected as dynamic (last
     * LIO). */
    double last_lidar_dynamic_ratio() const {
        return last_lidar_dynamic_ratio_;
    }
    /** @return Fraction of image pixels rejected as dynamic (last VIO). */
    double last_visual_dynamic_ratio() const {
        return last_visual_dynamic_ratio_;
    }

    /** @return Active LiDAR dynamic filter (may be null / pass-through). */
    frontend::DynamicFilterBase* lidar_dynamic_filter() {
        return lidar_dynamic_filter_.get();
    }
    /** @return Const LiDAR dynamic filter pointer (may be null). */
    const frontend::DynamicFilterBase* lidar_dynamic_filter() const {
        return lidar_dynamic_filter_.get();
    }
    /** @return Active visual dynamic filter (may be null). */
    frontend::DynamicFilterBase* visual_dynamic_filter() {
        return visual_dynamic_filter_.get();
    }

    /** @return Last confidence-gating outcome for fused observations. */
    const fusion::ObservationConfidence& last_observation_confidence() const {
        return last_observation_confidence_;
    }

    /** @return Last intensity-assist texture / inflation scale result. */
    const fusion::IntensityAssistResult& last_intensity_assist() const {
        return last_intensity_assist_;
    }

    /** @return Loop-closure manager (null when disabled). */
    loop_closure::LoopClosureManager* loop_closure_manager() {
        return loop_closure_manager_.get();
    }

    /** @return Map self-cleaning worker (null when disabled). */
    map::MapSelfCleaning* map_self_cleaning() {
        return map_cleaning_.get();
    }

    /** @return Relocalization manager (null when disabled). */
    relocalization::RelocalizationManager* relocalization_manager() {
        return relocalization_manager_.get();
    }

    /** @return Online map builder (null when mapping disabled). */
    mapping::MapBuilder* map_builder() {
        return map_builder_.get();
    }

    /**
     * @brief Load prior occupancy (`.xyz` or map package dir:
     * `prior.xyz`/`map.xyz`).
     * @param path_or_dir File path or map package directory.
     * @return false if path empty / unreadable.
     */
    bool LoadPriorMap(const std::string& path_or_dir);

    /**
     * @brief Load MapBuilder package (session + cloud + visual) and set reloc
     * prior.
     * @param directory Map package root directory.
     * @return false if directory unreadable.
     */
    bool LoadBuiltMap(const std::string& directory);

    /**
     * @brief Flush MapBuilder package to `mapping_output_dir` (or `directory`).
     * @param directory Output directory; empty uses
     * `config.mapping_output_dir`.
     * @return false when mapping is disabled or save fails.
     */
    bool SaveBuiltMap(const std::string& directory = {}) const;

    /**
     * @brief Copy sparse visual landmarks into MapBuilder (if mapping enabled).
     * @param visual Source visual map to import.
     */
    void ImportVisualMap(const map::VisualMap& visual);

private:
    bool ProcessLio(OdometryOutput* out);
    bool ProcessVio(OdometryOutput* out);
    void ApplyVioConfigToManager();
    void TrySubmitKeyFrame();
    void TryRequestRelocalization();
    void TryInjectRelocalization();
    void TryInjectLoopCorrection();
    void TryRequestCrossSessionRelocalization();
    /** @brief Push extrinsic/intrinsics into SemanticDynamicFilter if active.
     */
    void ConfigureSemanticProjection();

    void InitCoreModules();
    void ConfigureImuFromConfig();
    void RestartBackgroundWorkers();
    void InitVioIfEnabled();
    bool ApplyLioDynamicFilter(OdometryOutput* out);
    void SeedInitialVoxelMap();
    void BuildLioEsikfOptions(double now_sec,
                              backend::EsikfUpdateOptions* options);
    void UpdateLioMapAfterEsikf(double now_sec);
    void FillLioOutput(OdometryOutput* out, bool map_updated);

    LivoOdometryConfig config_;
    frontend::SensorSync sync_;
    frontend::Preprocess::SharedPtr preprocess_;
    frontend::ImuProcess::SharedPtr imu_process_;
    std::unordered_map<map::VoxelLocation, map::VoxelOctoTree*> voxel_hash_;
    map::VoxelMapManager::SharedPtr voxel_manager_;
    sensor::VoxelGrid<sensor::PointType> downsample_;
    VIOManager::SharedPtr vio_manager_;
    std::unique_ptr<camera::AbstractCamera> camera_;

    frontend::LidarMeasureGroup measures_;
    frontend::StatesGroup state_;
    frontend::StatesGroup state_propagated_;
    /** @brief Sole LIO scan + ESIKF scratch; bound into VoxelMapManager. */
    map::LioFrameBuffers lio_frame_;
    /** @brief Reusable VIO working image (avoids per-frame clone). */
    cv::Mat vio_image_work_;
    bool initialized_ = false;
    bool lidar_map_inited_ = false;
    bool gravity_align_finished_ = false;
    double first_lidar_time_ = -1.0;
    perception::DegeneracyResult last_degeneracy_;
    std::unique_ptr<frontend::DynamicFilterBase> lidar_dynamic_filter_;
    std::unique_ptr<frontend::DynamicFilterBase> visual_dynamic_filter_;
    /** @brief Last camera frame (for semantic filter when SyncOnlyLo has no
     * img). */
    cv::Mat last_image_;
    double last_lidar_dynamic_ratio_ = 0.0;
    double last_visual_dynamic_ratio_ = 0.0;
    perception::SensorHealthMonitor health_monitor_;
    fusion::ObservationConfidence last_observation_confidence_;
    fusion::IntensityAssistResult last_intensity_assist_;
    std::unique_ptr<loop_closure::LoopClosureManager> loop_closure_manager_;
    std::unique_ptr<map::MapSelfCleaning> map_cleaning_;
    std::unique_ptr<relocalization::RelocalizationManager> relocalization_manager_;
    std::unique_ptr<mapping::MapBuilder> map_builder_;
    int next_keyframe_id_ = 0;
    transform::Rigid3d last_keyframe_T_ = transform::Rigid3d::Identity();
    bool has_keyframe_ = false;
    bool cross_session_reloc_requested_ = false;
};

}  // namespace odometry
}  // namespace argus

#endif  // ARGUS_ODOMETRY_LIVO_ODOMETRY_HPP_

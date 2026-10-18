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
 * @file voxel_map.hpp
 * @brief VoxelMapManager and umbrella includes for the adaptive voxel LIO map.
 *
 * Related types live in one-file-per-type headers:
 * - voxel_location.hpp / voxel_plane.hpp / point_to_plane.hpp
 * - voxel_map_config.hpp / voxel_octo_tree.hpp / lio_frame_buffers.hpp
 * - voxel_body_cov.hpp
 */

#ifndef ARGUS_MAP_VOXEL_MAP_HPP_
#define ARGUS_MAP_VOXEL_MAP_HPP_

#include <cstdint>
#include <memory>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "argus/common/macros.hpp"
#include "argus/frontend/types.hpp"
#include "argus/map/lio_frame_buffers.hpp"
#include "argus/map/point_to_plane.hpp"
#include "argus/map/voxel_body_cov.hpp"
#include "argus/map/voxel_location.hpp"
#include "argus/map/voxel_map_config.hpp"
#include "argus/map/voxel_octo_tree.hpp"
#include "argus/map/voxel_plane.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace map {

using frontend::PointWithVar;
using frontend::pointWithVar;
using frontend::StatesGroup;
using sensor::PointCloud;
using sensor::PointCloudXYZI;
using sensor::PointType;
using sensor::PointXYZI;

/**
 * @brief Owns voxel octrees and builds point-to-plane residuals for LIO.
 *
 * Concurrent readers use `map_mutex()` (shared); writers take unique lock.
 */
class VoxelMapManager
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(VoxelMapManager);

    VoxelMapConfig
        config_setting_; /**< @brief Mutable voxel / plane settings. */
    int current_frame_id_ =
        0; /**< @brief Frame counter for plane bookkeeping. */
    /** @brief Shared hash with owner (LivoOdometry / wrapper); not owning. */
    std::unordered_map<VoxelLocation, VoxelOctoTree*>& voxel_map_;

    /**
     * @brief Active LIO frame buffers (owned or bound via BindFrameBuffers).
     * @return Non-null reference; default is internal storage.
     */
    LioFrameBuffers& frame() {
        return *frame_ptr_;
    }
    /** @copydoc frame() */
    const LioFrameBuffers& frame() const {
        return *frame_ptr_;
    }

    /**
     * @brief Point ESIKF / LIO scratch at externally owned buffers.
     * @param external Non-null; typically `LivoOdometry::lio_frame_`.
     *
     * Call once after construction when the orchestrator owns `LioFrameBuffers`.
     * Standalone tests may omit this and use the manager's owned storage.
     */
    void BindFrameBuffers(LioFrameBuffers* external);

    // --- Extrinsics (lidar ↔ IMU) ---
    Eigen::Matrix3d extR_ =
        Eigen::Matrix3d::Identity();                 /**< @brief Lidar→IMU R. */
    Eigen::Vector3d extT_ = Eigen::Vector3d::Zero(); /**< @brief Lidar→IMU t. */

    // --- Timing (optional profiling; not hot-path I/O) ---
    float build_residual_time_ = 0;
    float ekf_time_ = 0;
    float ave_build_residual_time_ = 0.0;
    float ave_ekf_time_ = 0.0;
    int scan_count_ = 0;

    // --- Filter / sliding state (mirrored into manager during ESIKF Update) ---
    StatesGroup state_;
    Eigen::Vector3d position_last_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_slide_position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond geo_quat_ = Eigen::Quaterniond::Identity();

    /**
     * @brief Construct manager bound to an external voxel hash.
     * @param config_setting Voxel / plane configuration (non-owning ref).
     * @param voxel_map Hash map owned by caller; manager mutates entries.
     */
    VoxelMapManager(
        VoxelMapConfig& config_setting,
        std::unordered_map<VoxelLocation, VoxelOctoTree*>& voxel_map);

    /**
     * @brief Legacy ESIKF update entry — implementation in `Argus::backend`
     *        (`EsikfStateFilter::Update`). Link `Argus::backend` when calling.
     * @param[in,out] state_propagated Propagated state; updated in place.
     */
    void StateEstimation(StatesGroup& state_propagated);

    /**
     * @brief Transform a lidar cloud by @p rot, @p t into @p trans_cloud.
     * @param rot Rotation (world←body or similar).
     * @param t Translation.
     * @param input_cloud Source cloud.
     * @param[out] trans_cloud Destination cloud (allocated by caller).
     */
    void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t,
                        const PointCloudXYZI::Ptr& input_cloud,
                        PointCloud<PointXYZI>::Ptr& trans_cloud);

    /** @brief Insert current downsampled world features into the voxel hash. */
    void BuildVoxelMap();

    /**
     * @brief Color for visualization from voxel hash of @p input_point.
     * @param input_point World position.
     * @return RGB in [0,1] as Vector3f.
     */
    Eigen::Vector3f RGBFromVoxel(const Eigen::Vector3d& input_point);

    /**
     * @brief Incremental map update from points with covariance.
     * @param input_points World points with variance / normal hints.
     */
    void UpdateVoxelMap(const std::vector<PointWithVar>& input_points);

    /**
     * @brief Erase root voxels (deletes octrees). Call under unique map lock
     *        or via MapSelfCleaning::Prune (takes the lock).
     * @param locations Root voxel keys to erase.
     * @return Number of voxels erased.
     */
    int EraseVoxels(const std::vector<VoxelLocation>& locations);

    /**
     * @brief Locations touched by the last UpdateVoxelMap.
     * @return Const ref to internal touch list.
     */
    const std::vector<VoxelLocation>& last_touched_voxels() const {
        return last_touched_;
    }

    /**
     * @brief When true, map updates continue under degeneracy flags.
     * @param v Enable flag.
     */
    void SetUpdateUnderDegeneracy(bool v) {
        update_under_degeneracy_ = v;
    }
    /** @return Whether map updates are allowed under degeneracy. */
    bool update_under_degeneracy() const {
        return update_under_degeneracy_;
    }

    /** @return Shared mutex guarding voxel_map_ reads/writes. */
    std::shared_mutex& map_mutex() const {
        return map_mutex_;
    }

    /**
     * @brief Build point-to-plane residuals (OpenMP over points).
     * @param[in,out] pv_list Points with variance; may be filtered in place.
     * @param[out] ptpl_list Filled point-to-plane associations.
     */
    void BuildResidualListOMP(std::vector<PointWithVar>& pv_list,
                              std::vector<PointToPlane>& ptpl_list);

    /**
     * @brief Recurse one octree for a single point residual.
     * @param[in,out] pv Query point.
     * @param current_octo Octree node to search.
     * @param current_layer Depth index.
     * @param[out] is_sucess True if a valid plane match was found.
     * @param[out] prob Association probability / weight.
     * @param[out] single_ptpl Filled residual when successful.
     */
    void BuildSingleResidual(PointWithVar& pv,
                             const VoxelOctoTree* current_octo,
                             int current_layer, bool& is_sucess, double& prob,
                             PointToPlane& single_ptpl);

    /**
     * @brief Collect updated planes for external visualization (no ROS).
     * @param[out] planes Destination buffer (cleared then filled).
     */
    void CollectUpdatePlanes(std::vector<VoxelPlane>* planes) const;

    /** @brief Slide / prune map around latest pose when enabled in config. */
    void MapSliding();

    /**
     * @brief Delete voxels outside an axis-aligned index box.
     * @param x_max,x_min,y_max,y_min,z_max,z_min Inclusive voxel-index bounds.
     */
    void ClearMemOutOfMap(const int& x_max, const int& x_min, const int& y_max,
                          const int& y_min, const int& z_max, const int& z_min);

private:
    void GetUpdatePlane(const VoxelOctoTree* current_octo,
                        int pub_max_voxel_layer,
                        std::vector<VoxelPlane>& plane_list) const;
    void MapJet(double v, double vmin, double vmax, uint8_t& r, uint8_t& g,
                uint8_t& b) const;

    /** @brief Default buffers when BindFrameBuffers is not used. */
    LioFrameBuffers owned_frame_;
    /** @brief Active buffers; defaults to `owned_frame_`. */
    LioFrameBuffers* frame_ptr_ = &owned_frame_;

    mutable std::shared_mutex map_mutex_;
    std::vector<VoxelLocation> last_touched_;
    bool update_under_degeneracy_ = false;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_VOXEL_MAP_HPP_

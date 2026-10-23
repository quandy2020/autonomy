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
 * @file map_builder.hpp
 * @brief Concrete map builder: session traj + ikd map + reloc prior export.
 */

#ifndef ARGUS_MAPPING_MAP_BUILDER_HPP_
#define ARGUS_MAPPING_MAP_BUILDER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "argus/common/macros.hpp"
#include "argus/map/ikd_tree.hpp"
#include "argus/map/map_session.hpp"
#include "argus/map/occupancy_grid_builder.hpp"
#include "argus/map/visual_map.hpp"
#include "argus/mapping/map_builder_interface.hpp"
#include "argus/mapping/map_package.hpp"
#include "argus/relocalization/loop_closure/types.hpp"
#include "argus/relocalization/prior_map.hpp"

namespace argus {
namespace mapping {

struct MapBuilderConfig {
    std::string session_id = "map";
    int node_id_base = 0;
    double ikd_downsample_resolution = 0.2;
    double prior_resolution = 0.5;
    /** @brief Max points kept in exported XYZ (0 = unlimited). */
    std::size_t max_export_points = 500000;
    /** @brief Fuse keyframes into 2.5D OccupancyGridMap (default off). */
    bool build_occupancy_grid = false;
    map::OccupancyGridBuilderOptions occupancy_grid;
};

/** @brief Incremental ikd map + session traj exporter for on-disk packages. */
class MapBuilder : public MapBuilderInterface
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(MapBuilder);

    /**
     * @brief Construct builder with export and downsampling options.
     * @param config Session id, resolutions, and export caps.
     */
    explicit MapBuilder(MapBuilderConfig config = {});

    /** @return Active builder configuration. */
    const MapBuilderConfig& config() const {
        return config_;
    }

    /**
     * @brief Clear map state and start a new session id.
     * @param session_id Logical session identifier for exports.
     */
    void Reset(const std::string& session_id) override;
    /**
     * @brief Append a keyframe to trajectory and ikd map.
     * @param kf Pose, stamp, and optional body cloud.
     */
    void AddKeyFrame(const MapBuilderKeyFrame& kf) override;
    /**
     * @brief Write map package to disk.
     * @param directory Output package root.
     * @return false on I/O or validation error.
     */
    bool Save(const std::string& directory) const override;
    /**
     * @brief Load map package from disk.
     * @param directory Package root directory.
     * @return false if layout invalid or unreadable.
     */
    bool Load(const std::string& directory) override;

    /** @return Const session registry embedded in the package. */
    const map::MapSession& session() const override {
        return session_;
    }
    /** @return Mutable session registry. */
    map::MapSession& mutable_session() {
        return session_;
    }

    /** @return Number of stored keyframes. */
    std::size_t keyframe_count() const override {
        return keyframes_.size();
    }
    /** @return Point count in the ikd map. */
    std::size_t map_point_count() const override {
        return ikd_.size();
    }

    /** @return Mutable incremental kd-tree (LTA history inject). */
    map::IkdTree& ikd() {
        return ikd_;
    }
    /** @return Const incremental kd-tree map. */
    const map::IkdTree& ikd() const {
        return ikd_;
    }

    /** @return Mutable sparse visual landmark map. */
    map::VisualMap& visual_map() {
        return visual_;
    }
    /** @return Const visual map. */
    const map::VisualMap& visual_map() const {
        return visual_;
    }

    /**
     * @brief Insert a landmark (takes ownership of a new VisualPoint).
     * @param pos World position (m).
     * @param normal Optional surface normal; zero if unknown.
     */
    void AddVisualLandmark(
        const Eigen::Vector3d& pos,
        const Eigen::Vector3d& normal = Eigen::Vector3d::Zero());

    /**
     * @brief Copy landmark positions/normals from an external VisualMap.
     * @param src Source map to merge from.
     */
    void ImportVisualLandmarks(const map::VisualMap& src);

    /**
     * @brief Build occupancy prior for RelocalizationManager::SetPriorMap.
     * @return Voxel prior at `config.prior_resolution`, or nullptr if empty.
     */
    relocalization::PriorOccupancyMap::SharedPtr ExportPriorMap() const;

    /** @return Last loaded / saved package meta (default until Save/Load). */
    const MapPackageMeta& package_meta() const {
        return package_meta_;
    }

    /**
     * @brief In-memory keyframes (clouds filled when keyframes/ saved or live).
     * @return Const keyframe vector.
     */
    const std::vector<MapBuilderKeyFrame>& keyframes() const {
        return keyframes_;
    }

    /**
     * @brief Export package keyframes as loop-closure KeyFrames (body cloud).
     *
     * Sets `id = node_id_base + local_id` for cross-session SC matching.
     * World-frame stored clouds are converted to body via \(T_{wb}^{-1}\).
     *
     * @return Non-empty-cloud keyframes suitable for RegisterPriorSession.
     */
    std::vector<loop_closure::KeyFrame> ExportLoopKeyFrames() const;

    /**
     * @brief 2.5D occupancy map when `build_occupancy_grid` is enabled.
     * @return Shared map or nullptr.
     */
    map::OccupancyGridMap::SharedPtr occupancy_grid() const;

private:
    bool SaveCloudXyz(const std::string& path) const;
    bool LoadCloudXyz(const std::string& path);
    bool SavePriorXyz(const std::string& path) const;
    bool SaveTrajectoryTum(const std::string& path) const;
    bool SaveKeyframeClouds(const std::string& directory) const;
    bool LoadKeyframeClouds(const std::string& directory);
    bool SaveOccupancyPng(const std::string& path) const;
    sensor::PointCloudXYZI CollectMapCloud() const;

    MapBuilderConfig config_;
    map::MapSession session_;
    map::IkdTree ikd_;
    map::VisualMap visual_;
    std::vector<MapBuilderKeyFrame> keyframes_;
    mutable MapPackageMeta package_meta_;
    std::unique_ptr<map::OccupancyGridBuilder> occupancy_;
};

}  // namespace mapping
}  // namespace argus

#endif  // ARGUS_MAPPING_MAP_BUILDER_HPP_

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
 * @file lidar_odometry.hpp
 * @brief Scan-to-map lidar frontend. Pose is refined with Ceres point-to-plane.
 *
 * Registry names `"lio"` and `"livo"`. LIVO consumes an optional absolute pose
 * prior from the visual tracker; the prior is not a second optimizer.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_LIDAR_LIDAR_ODOMETRY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_LIDAR_LIDAR_ODOMETRY_HPP_

#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

#include "autonomy/localization/atlas/frontend/frontend_base.hpp"
#include "autonomy/localization/atlas/frontend/lidar/scan_filter.hpp"
#include "autonomy/localization/atlas/map/occupancy/occupancy_map.hpp"
#include "autonomy/localization/atlas/sensor/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @brief LiDAR odometry on a voxel map. Mapping inserts scans; localization does not.
 */
class LidarOdometry : public FrontendBase {
public:
    bool Init(const AtlasConfig& config) override;
    bool Process(const SensorData& data) override;
    bool GetResult(OdometryResult* out) const override;
    void Reset() override;
    FrontendMode Mode() const override;
    bool ConsumePendingKeyframe(Keyframe* out) override;

    /**
     * @brief Loose visual prior for the next scan (LIVO). Cleared after use.
     * @param T_wb Visual \(T_{wb}\).
     * @param sqrt_info Square-root information.
     */
    void SetPosePrior(const SE3& T_wb, double sqrt_info);

    /**
     * @brief Replace the voxel map. Used by localization and relocalization.
     * @param cloud World-frame points.
     */
    void LoadMap(const PointCloud& cloud);

    /**
     * @brief Write point tiles and the occupancy grid under @p directory.
     * @param directory Map directory.
     * @return false when the map is empty or a tile cannot be written.
     */
    bool SaveMap(const std::string& directory) const;

    /**
     * @brief Replace the voxel map and occupancy grid from @p directory.
     * @param directory Map directory produced by SaveMap.
     * @return false when no tile was read.
     */
    bool LoadMapDirectory(const std::string& directory);

    /**
     * @brief Voxel map as `sensor_msgs/PointCloud2` in the map frame.
     * @param[out] cloud Output message.
     * @return false when the map is empty.
     */
    bool FillDenseCloud(PointCloud2* cloud) const;

    /**
     * @brief Latest 2D occupancy grid.
     * @param[out] grid `map_msgs/OccupancyGrid`.
     * @return false when no ray has been written.
     */
    bool FillOccupancyGrid(automsgs::msgs::map_msgs::OccupancyGrid* grid) const;

    /**
     * @brief Clear the grid and raycast every stored lidar keyframe again.
     *
     * Called after a visual loop so the navigation grid matches the keyframes
     * kept in this frontend.
     */
    void RebuildOccupancy();

private:
    struct VoxelKey {
        int x = 0;
        int y = 0;
        int z = 0;
        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    struct VoxelHash {
        std::size_t operator()(const VoxelKey& key) const {
            return (static_cast<std::size_t>(key.x) * 73856093u) ^
                   (static_cast<std::size_t>(key.y) * 19349663u) ^
                   (static_cast<std::size_t>(key.z) * 83492791u);
        }
    };

    VoxelKey KeyOf(const Vec3& point) const;
    void InsertPoint(const Vec3& point);
    bool FitPlane(const Vec3& query, Vec3* plane_point, Vec3* plane_normal) const;
    SE3 Predict(const SensorData& data) const;
    void MaybeQueueKeyframe(double stamp, const PointCloud& body_cloud);
    void Deskew(PointCloud* scan, const SensorData& data, double stamp) const;
    int ScorePose(const PointCloud& scan, const SE3& pose) const;
    bool SearchYaw(const PointCloud& scan, SE3* pose) const;

    AtlasConfig config_;
    std::unordered_map<VoxelKey, std::vector<Vec3>, VoxelHash> voxels_;
    std::size_t map_points_ = 0;
    SE3 T_wb_ = SE3Identity();
    bool has_pose_ = false;
    double last_stamp_ = 0.0;
    OdometryResult result_;
    std::deque<Keyframe> pending_;
    int next_keyframe_id_ = 0;
    SE3 last_keyframe_pose_ = SE3Identity();
    bool has_keyframe_ = false;

    bool has_prior_ = false;
    SE3 prior_ = SE3Identity();
    double prior_sqrt_info_ = 0.0;
    bool reloc_done_ = false;
    OccupancyMap occupancy_;
    std::vector<Keyframe> lidar_keyframes_;
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_LIDAR_LIDAR_ODOMETRY_HPP_

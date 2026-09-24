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
 * @file faster_lio_stack.hpp
 * @brief Atlas-owned Faster-LIO stack: ESKF + ivox, Ceres loop/pose graph,
 *        G2P5 occupancy, and NDT localization on a tiled map.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_LIDAR_FASTER_LIO_STACK_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_LIDAR_FASTER_LIO_STACK_HPP_

#include <functional>
#include <memory>
#include <vector>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>

#include "autonomy/localization/atlas/common/config.hpp"
#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/sensor/imu/types.hpp"
#include "autonomy/localization/atlas/sensor/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

struct LidarConstraintEdge {
    double x0 = 0.0;
    double y0 = 0.0;
    double z0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    double z1 = 0.0;
};

struct LidarConstraintGraph {
    std::vector<LidarConstraintEdge> odom;
    std::vector<LidarConstraintEdge> loops;
    std::vector<LidarConstraintEdge> reloc;
};

class FasterLioStack {
public:
    FasterLioStack();
    ~FasterLioStack();

    FasterLioStack(const FasterLioStack&) = delete;
    FasterLioStack& operator=(const FasterLioStack&) = delete;

    /**
     * @brief Build ESKF mapping, or NDT localization when a map directory is set.
     * @return false when the YAML path is empty or initialization throws.
     */
    bool Init(const AtlasConfig& config);

    void ProcessImu(const sensor::imu::Measurement& measurement);

    /**
     * @brief Deskew, ESKF update, and either loop/G2P5 or NDT + pose graph.
     * @param[out] T_wb Map-from-IMU pose.
     */
    bool ProcessCloud(const SensorData& data, SE3* T_wb);

    bool FillDenseCloud(PointCloud2* cloud) const;
    /// Current deskewed scan in the loop-corrected map frame.
    bool FillRegisteredCloud(PointCloud2* cloud) const;
    bool FillOccupancyGrid(automsgs::msgs::map_msgs::OccupancyGrid* grid) const;
    bool FillLidarConstraints(LidarConstraintGraph* graph) const;

    /// Write the loop-optimized cloud as a tiled map plus `global.pcd`.
    bool SaveMap(const std::string& directory) const;

    /// NDT localization was started from a map directory in `Init`.
    bool localizing() const;

    /// Loose visual pose for the next ESKF scan. Consumed once.
    void SetVisualPrior(const SE3& T_wb, double weight);

    /// G2P5 calls this from its render thread when a new grid is ready.
    void SetOccupancyCallback(
        std::function<void(const automsgs::msgs::map_msgs::OccupancyGrid&)>
            callback);

    bool active() const { return active_; }

private:
    struct Impl;
    void LoadSavedProducts(const std::string& directory);

    std::unique_ptr<Impl> impl_;
    bool active_ = false;
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_LIDAR_FASTER_LIO_STACK_HPP_

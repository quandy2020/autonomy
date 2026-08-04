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

#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "autonomy/common/math/polygon2d.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon.pb.h>
#include <automsgs/msgs/geometry_msgs/point32.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>
#include "autonomy/exploration/proto/exploration_options.pb.h"
#include "autonomy/exploration/viewpoint/camera_model.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy {
namespace exploration {

/**
 * @file planning_env.hpp
 * @brief RGBD planning environment with Bayesian 2D occupancy + coverage layer.
 */

/**
 * @class PlanningEnv
 * @brief Depth fusion, frontier extraction, and coverage target management.
 */
class PlanningEnv
{
public:
    /**
     * @brief Constructor with exploration options.
     * @param options Exploration options
     */
    explicit PlanningEnv(const proto::ExplorationOptions& options);

    /**
     * @brief Update parameters and reinitialize the costmap.
     * @param options Exploration options
     */
    void SetOptions(const proto::ExplorationOptions& options);

    /**
     * @brief Set the exploration boundary polygon.
     * @param area Boundary in the map frame
     */
    void SetExplorationArea(const automsgs::msgs::geometry_msgs::Polygon& area);

    /**
     * @brief Update robot pose from odometry and refresh coverage.
     * @param odom Robot odometry
     */
    void UpdateOdometry(const automsgs::msgs::nav_msgs::Odometry& odom);

    /**
     * @brief Fuse a depth frame and refresh coverage targets.
     * @param depth Depth image
     * @param info Camera intrinsics
     * @param map_t_camera Extrinsic transform from camera to map
     */
    void UpdateDepth(const automsgs::msgs::sensor_msgs::Image& depth,
                     const automsgs::msgs::sensor_msgs::CameraInfo& info,
                     const automsgs::msgs::geometry_msgs::Transform& map_t_camera);

    /**
     * @brief Mark cells covered by the TF camera FoV (coverage layer only).
     */
    void UpdateCoverageFromTf();

    /**
     * @brief Get current coverage targets (frontier clusters / unknowns).
     * @return Target points in the map frame
     */
    const std::vector<automsgs::msgs::geometry_msgs::Point>& targets() const
    {
        return targets_;
    }

    /**
     * @brief Get uncovered flags aligned with targets().
     * @return Uncovered flags
     */
    const std::vector<bool>& uncovered() const { return uncovered_; }

    /**
     * @brief Get extracted frontier points (pre-cluster).
     * @return Frontier points
     */
    const std::vector<automsgs::msgs::geometry_msgs::Point>& frontiers() const
    {
        return frontiers_;
    }

    /**
     * @brief Set map / camera frames for TF-based coverage (optional).
     * @param map_frame Map frame id
     * @param camera_frame Camera frame id
     */
    void SetFrames(const std::string& map_frame,
                   const std::string& camera_frame)
    {
        camera_.SetFrames(map_frame, camera_frame);
    }

    /**
     * @brief Access the occupancy costmap (lethal / free / unknown).
     * @return Const costmap reference
     */
    const map::costmap_2d::Costmap2D& costmap() const { return costmap_; }

    /**
     * @brief Inflated costmap used for collision queries.
     * @return Const inflated costmap reference
     */
    const map::costmap_2d::Costmap2D& inflated_costmap() const
    {
        return inflated_;
    }

    /**
     * @brief Check whether a map cell is lethal / inflated-occupied.
     * @param x Query x [m]
     * @param y Query y [m]
     * @return true if occupied for planning
     */
    bool IsOccupied(double x, double y) const;

    /**
     * @brief Check whether a map cell is free (not lethal, known free).
     * @param x Query x [m]
     * @param y Query y [m]
     * @return true if free
     */
    bool IsFree(double x, double y) const;

    /**
     * @brief Whether a cell has been marked covered by FoV.
     * @param x Query x [m]
     * @param y Query y [m]
     * @return true if covered
     */
    bool IsCovered(double x, double y) const;

    /**
     * @brief Grid A* path between two world points on the inflated map.
     * @param from_x Start x [m]
     * @param from_y Start y [m]
     * @param to_x Goal x [m]
     * @param to_y Goal y [m]
     * @param path Output polyline (empty if unreachable)
     * @return Path length [m], or inf if fail
     */
    double PlanPathAStar(double from_x, double from_y, double to_x, double to_y,
                         std::vector<automsgs::msgs::geometry_msgs::Point>* path) const;

    /**
     * @brief Get robot x.
     * @return Robot x [m]
     */
    double robot_x() const { return robot_x_; }

    /**
     * @brief Get robot y.
     * @return Robot y [m]
     */
    double robot_y() const { return robot_y_; }

    /**
     * @brief Get robot z.
     * @return Robot z [m]
     */
    double robot_z() const { return robot_z_; }

    /**
     * @brief Get robot yaw.
     * @return Robot yaw [rad]
     */
    double robot_yaw() const { return robot_yaw_; }

    /**
     * @brief Check whether a point lies inside the exploration polygon.
     * @param x Query x [m]
     * @param y Query y [m]
     * @return true if inside (or polygon is empty/invalid)
     */
    bool IsInExplorationArea(double x, double y) const;

    /**
     * @brief Export the costmap as OccupancyGrid.
     * @param frame_id Header frame id
     * @return Occupancy grid message
     */
    automsgs::msgs::map_msgs::OccupancyGrid GetOccupancyGrid(
        const std::string& frame_id) const;

private:
    /**
     * @brief Allocate / reset the rolling costmap from options.
     */
    void InitCostmap();

    /**
     * @brief Archive local cells then recenter the rolling window.
     * @param x Robot x [m]
     * @param y Robot y [m]
     */
    void RollCostmapIfNeeded(double x, double y);

    /**
     * @brief Dump local log-odds / covered into the global hash.
     */
    void ArchiveLocalToGlobal();

    /**
     * @brief Restore overlapping global cells into the local window.
     */
    void RestoreGlobalToLocal();

    /**
     * @brief Apply log-odds hit update at a world point.
     * @param x World x [m]
     * @param y World y [m]
     */
    void UpdateHit(double x, double y);

    /**
     * @brief Apply log-odds miss update along a ray until hit.
     * @param ox Ray origin x [m]
     * @param oy Ray origin y [m]
     * @param tx Ray end x [m]
     * @param ty Ray end y [m]
     */
    void UpdateMissRay(double ox, double oy, double tx, double ty);

    /**
     * @brief Sync log_odds_ → costmap_ cell costs.
     */
    void SyncCostmapFromLogOdds();

    /**
     * @brief Rebuild inflated_ from lethal cells + collision_radius.
     */
    void RebuildInflation();

    /**
     * @brief Extract free–unknown frontiers.
     * @param frontiers Output frontier points
     */
    void ExtractFrontiers(
        std::vector<automsgs::msgs::geometry_msgs::Point>* frontiers) const;

    /**
     * @brief Cluster frontiers into gain targets.
     * @param frontiers Input frontier cells
     * @param targets Output cluster centroids
     * @param uncovered Output uncovered flags
     */
    void ClusterFrontiers(
        const std::vector<automsgs::msgs::geometry_msgs::Point>& frontiers,
        std::vector<automsgs::msgs::geometry_msgs::Point>* targets,
        std::vector<bool>* uncovered) const;

    /**
     * @brief Extract unknown cells as coverage targets.
     * @param targets Output target points
     * @param uncovered Output uncovered flags
     */
    void ExtractUnknownTargets(
        std::vector<automsgs::msgs::geometry_msgs::Point>* targets,
        std::vector<bool>* uncovered) const;

    /**
     * @brief Mark FoV-visible cells using an explicit camera extrinsic.
     * @param map_t_camera Extrinsic of camera in map
     */
    void MarkCoveredFromExtrinsic(
        const automsgs::msgs::geometry_msgs::Transform& map_t_camera);

    /**
     * @brief Mark FoV-visible cells using cached / TF camera pose (single lookup).
     */
    void MarkCoveredFromTf();

    /**
     * @brief Project depth pixels and Bayesian-update occupancy.
     * @param depth Depth image
     * @param info Camera intrinsics
     * @param map_t_camera Extrinsic transform from camera to map
     */
    void FuseDepthFrame(const automsgs::msgs::sensor_msgs::Image& depth,
                        const automsgs::msgs::sensor_msgs::CameraInfo& info,
                        const automsgs::msgs::geometry_msgs::Transform& map_t_camera);

    /**
     * @brief Rebuild targets_ / frontiers_ from the costmap.
     */
    void RefreshTargets();

    /**
     * @brief Set a default square exploration polygon from options.
     */
    void SetDefaultExplorationArea();

    /**
     * @brief Global hash key from discrete cell indices.
     * @param gx Global cell x
     * @param gy Global cell y
     * @return Hash key
     */
    static int64_t GlobalKey(int gx, int gy);

    /**
     * @brief Convert world to global cell indices.
     * @param wx World x [m]
     * @param wy World y [m]
     * @param gx Output gx
     * @param gy Output gy
     */
    void WorldToGlobal(double wx, double wy, int* gx, int* gy) const;

    /**
     * @brief Apply log-odds delta and sync one cell.
     * @param index Flat local index
     * @param delta Log-odds delta
     */
    void ApplyLogOdds(size_t index, float delta);

    proto::ExplorationOptions options_;   //!< @brief exploration options
    CameraModel camera_;                  //!< @brief FoV camera model
    map::costmap_2d::Costmap2D costmap_;  //!< @brief rolling occupancy
    map::costmap_2d::Costmap2D inflated_; //!< @brief inflated for collision
    ::autonomy::common::math::Polygon2d exploration_polygon_;
    automsgs::msgs::geometry_msgs::Polygon exploration_area_;

    std::vector<float> log_odds_;   //!< @brief Bayesian log-odds (0=unknown)
    std::vector<uint8_t> covered_;  //!< @brief FoV coverage flags
    std::unordered_map<int64_t, float> global_log_odds_;  //!< @brief archive
    std::unordered_map<int64_t, uint8_t> global_covered_;  //!< @brief archive

    std::vector<automsgs::msgs::geometry_msgs::Point> targets_;
    std::vector<bool> uncovered_;
    std::vector<automsgs::msgs::geometry_msgs::Point> frontiers_;

    double robot_x_{0.0};
    double robot_y_{0.0};
    double robot_z_{0.0};
    double robot_yaw_{0.0};
    bool has_odom_{false};
    automsgs::msgs::geometry_msgs::Transform
        last_map_t_camera_{};             //!< @brief last camera extrinsic
    bool has_map_t_camera_{false};        //!< @brief whether extrinsic is valid
};

}  // namespace exploration
}  // namespace autonomy

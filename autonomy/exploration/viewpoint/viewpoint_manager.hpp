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

#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/exploration/planning_env.hpp"
#include "autonomy/exploration/proto/exploration_options.pb.h"
#include "autonomy/exploration/viewpoint/camera_model.hpp"

namespace autonomy {
namespace exploration {

/**
 * @file viewpoint_manager.hpp
 * @brief Local viewpoint grid with yaw sampling for RGBD FoV coverage.
 */

/**
 * @brief Coarse cell coverage status for hierarchical exploration.
 */
enum class CellStatus : std::uint8_t {
    kUnseen = 0,     //!< @brief cell has not been observed
    kExploring = 1,  //!< @brief cell still has exploration gain
    kCovered = 2,    //!< @brief cell is considered covered
};

/**
 * @brief Candidate observation pose with FoV coverage score.
 */
struct Viewpoint {
    double x{0.0};          //!< @brief position x [m]
    double y{0.0};          //!< @brief position y [m]
    double z{0.0};          //!< @brief position z [m]
    double yaw{0.0};        //!< @brief yaw [rad]
    double pitch{0.0};      //!< @brief pitch [rad]
    int id{-1};             //!< @brief unique viewpoint id
    double gain{0.0};       //!< @brief coverage gain
    bool visited{false};    //!< @brief already visited flag
    bool selected{false};   //!< @brief selected in the current plan
    bool in_los{true};      //!< @brief LOS to robot is clear
    bool connected{true};   //!< @brief neighboring LOS connectivity
    int cell_index{-1};     //!< @brief associated GridWorld cell index

    /**
     * @brief Convert to PoseStamped.
     * @param frame_id Header frame id
     * @return Pose stamped message
     */
    commsgs::geometry_msgs::PoseStamped ToPoseStamped(
        const std::string& frame_id) const;
};

/**
 * @class ViewpointManager
 * @brief Builds and scores a rolling local viewpoint lattice around the robot.
 */
class ViewpointManager
{
public:
    /**
     * @brief Constructor with exploration options.
     * @param options Exploration options
     */
    explicit ViewpointManager(const proto::ExplorationOptions& options);

    /**
     * @brief Update parameters from exploration options.
     * @param options Exploration options
     */
    void SetOptions(const proto::ExplorationOptions& options);

    /**
     * @brief Update robot pose and rebuild the local viewpoint grid.
     * @param x Robot x [m]
     * @param y Robot y [m]
     * @param z Robot z [m]
     * @param yaw Robot yaw [rad]
     */
    void UpdateRobotPosition(double x, double y, double z, double yaw);

    /**
     * @brief Rescore viewpoints using the planning environment and cell status.
     * @param env Planning environment
     * @param cell_status GridWorld cell statuses
     * @param grid_cols Number of GridWorld columns
     * @param cell_size GridWorld cell size [m]
     * @param origin_x GridWorld origin x [m]
     * @param origin_y GridWorld origin y [m]
     */
    void UpdateFromEnv(const PlanningEnv& env,
                       const std::vector<CellStatus>& cell_status,
                       int grid_cols, double cell_size, double origin_x,
                       double origin_y);

    /**
     * @brief Get all viewpoints in the current local lattice.
     * @return Viewpoint list
     */
    const std::vector<Viewpoint>& viewpoints() const { return viewpoints_; }

    /**
     * @brief Get unvisited viewpoints above a minimum gain, sorted by gain.
     * @param min_gain Minimum gain threshold
     * @return Candidate viewpoints ordered by descending gain
     */
    std::vector<Viewpoint> GetCandidates(double min_gain) const;

    /**
     * @brief Build a viewpoint at the current robot pose.
     * @return Robot viewpoint
     */
    Viewpoint GetRobotViewpoint() const;

    /**
     * @brief Check collision against the planning occupancy map.
     * @param x Query x [m]
     * @param y Query y [m]
     * @return true if occupied
     */
    bool IsCollision(double x, double y) const;

    /**
     * @brief Map a position to a GridWorld cell index.
     * @param x Query x [m]
     * @param y Query y [m]
     * @param grid_cols Number of columns
     * @param cell_size Cell size [m]
     * @param origin_x Origin x [m]
     * @param origin_y Origin y [m]
     * @return Cell index, or -1 if invalid
     */
    int CellIndexForPosition(double x, double y, int grid_cols,
                             double cell_size, double origin_x,
                             double origin_y) const;

    /**
     * @brief Mark a viewpoint as visited by id.
     * @param id Viewpoint id
     */
    void MarkVisited(int id);

    /**
     * @brief Mark viewpoints near a position as visited.
     * @param x Center x [m]
     * @param y Center y [m]
     * @param radius Radius [m]
     */
    void MarkVisitedNear(double x, double y, double radius);

private:
    /**
     * @brief Rebuild the local (x, y, yaw) viewpoint lattice.
     */
    void RebuildGrid();

    /**
     * @brief Score each viewpoint against coverage targets.
     * @param env Planning environment
     */
    void ScoreViewpoints(const PlanningEnv& env);

    /**
     * @brief Update in_los / connected flags using costmap LOS.
     * @param env Planning environment
     */
    void UpdateLineOfSightFlags(const PlanningEnv& env);

    proto::ExplorationOptions options_;  //!< @brief exploration options
    CameraModel camera_;                 //!< @brief FoV camera model
    std::vector<Viewpoint> viewpoints_;  //!< @brief local viewpoint lattice
    double robot_x_{0.0};                //!< @brief robot x [m]
    double robot_y_{0.0};                //!< @brief robot y [m]
    double robot_z_{0.0};                //!< @brief robot z [m]
    double robot_yaw_{0.0};              //!< @brief robot yaw [rad]
    double grid_origin_x_{0.0};          //!< @brief lattice origin x [m]
    double grid_origin_y_{0.0};          //!< @brief lattice origin y [m]
    int next_id_{0};                     //!< @brief next viewpoint id
    const PlanningEnv* env_{nullptr};    //!< @brief non-owning env pointer
};

}  // namespace exploration
}  // namespace autonomy

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
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/exploration/planning_env.hpp"
#include "autonomy/exploration/proto/exploration_options.pb.h"
#include "autonomy/exploration/viewpoint/viewpoint_manager.hpp"

namespace autonomy {
namespace exploration {

/**
 * @file grid_world.hpp
 * @brief Coarse global cell status for hierarchical exploration.
 */

/**
 * @class GridWorld
 * @brief Tracks UNSEEN / EXPLORING / COVERED cells over the exploration area.
 */
class GridWorld
{
public:
    /**
     * @brief Constructor with exploration options.
     * @param options Exploration options
     */
    explicit GridWorld(const proto::ExplorationOptions& options);

    /**
     * @brief Update grid sizing parameters from options.
     * @param options Exploration options
     */
    void SetOptions(const proto::ExplorationOptions& options);

    /**
     * @brief Recenter the grid origin around a world position.
     * @param x World x [m]
     * @param y World y [m]
     */
    void SetOrigin(double x, double y);

    /**
     * @brief Update cell statuses from viewpoint gains and area membership.
     * @param viewpoints Scored viewpoints
     * @param env Planning environment (for exploration area tests)
     */
    void UpdateCellStatus(const ViewpointManager& viewpoints,
                          const PlanningEnv& env);

    /**
     * @brief Get all cell statuses.
     * @return Cell status vector (row-major)
     */
    const std::vector<CellStatus>& cell_status() const { return status_; }

    /**
     * @brief Get number of columns.
     * @return Column count
     */
    int cols() const { return cols_; }

    /**
     * @brief Get number of rows.
     * @return Row count
     */
    int rows() const { return rows_; }

    /**
     * @brief Get cell size.
     * @return Cell size [m]
     */
    double cell_size() const { return cell_size_; }

    /**
     * @brief Get grid origin x.
     * @return Origin x [m]
     */
    double origin_x() const { return origin_x_; }

    /**
     * @brief Get grid origin y.
     * @return Origin y [m]
     */
    double origin_y() const { return origin_y_; }

    /**
     * @brief Collect EXPLORING cells outside the local radius.
     * @param robot_x Robot x [m]
     * @param robot_y Robot y [m]
     * @param nearby_radius Local cell radius to exclude
     * @return Global exploring cell indices
     */
    std::vector<int> GetExploringCells(double robot_x, double robot_y,
                                       int nearby_radius) const;

    /**
     * @brief Get the center of a cell.
     * @param cell_index Cell index
     * @return Cell center point
     */
    commsgs::geometry_msgs::Point CellCenter(int cell_index) const;

    /**
     * @brief Check whether a cell is within nearby_radius of the robot cell.
     * @param cell_index Cell index
     * @param robot_x Robot x [m]
     * @param robot_y Robot y [m]
     * @param nearby_radius Chebyshev radius in cells
     * @return true if nearby
     */
    bool IsNearbyCell(int cell_index, double robot_x, double robot_y,
                      int nearby_radius) const;

    /**
     * @brief Fraction of cells marked COVERED.
     * @return Coverage progress in [0, 1]
     */
    float CoverageProgress() const;

private:
    /**
     * @brief Convert world coordinates to a cell index.
     * @param x World x [m]
     * @param y World y [m]
     * @return Cell index, or -1 if out of bounds
     */
    int CellIndex(double x, double y) const;

    /**
     * @brief Allocate status_ / cell_gain_ buffers.
     */
    void EnsureSize();

    proto::ExplorationOptions options_;  //!< @brief exploration options
    int cols_{20};                       //!< @brief number of columns
    int rows_{20};                       //!< @brief number of rows
    double cell_size_{4.0};              //!< @brief cell size [m]
    double origin_x_{-40.0};             //!< @brief origin x [m]
    double origin_y_{-40.0};             //!< @brief origin y [m]
    std::vector<CellStatus> status_;     //!< @brief per-cell status
    std::vector<double> cell_gain_;      //!< @brief per-cell max viewpoint gain
};

}  // namespace exploration
}  // namespace autonomy

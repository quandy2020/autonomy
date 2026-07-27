/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include <optional>
#include <utility>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/types.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

struct PathfinderOptions {
    double widthMeters{0.};
    double heightMeters{0.};
    double gridSize{0.5};
    double turnPenalty{2.8};
    double diagonalPenalty{0.8};
    int smoothClearance{2};
    int dilatePasses{3};
    double marginLeft{0.05};
    double marginRight{0.05};
    double marginTop{0.};
    double marginBottom{0.05};
};

class Pathfinder
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(Pathfinder)

    explicit Pathfinder(const PathfinderOptions& options);

    void SetObstacles(const std::vector<ObstaclePolygon>& obstacles);

    double widthMeters() const { return options_.widthMeters; }
    double heightMeters() const { return options_.heightMeters; }

    // Coordinates in fractional [0,1] range.
    std::optional<std::vector<std::pair<double, double>>> FindPath(
        const std::pair<double, double>& start, const std::pair<double, double>& end);

private:
    struct Cell {
        int col{0};
        int row{0};
    };

    std::vector<std::vector<bool>> BuildObstacleGrid(
        const std::vector<ObstaclePolygon>& obstacles);
    std::optional<Cell> NearestWalkableCell(const Cell& cell,
                                            const std::vector<std::vector<bool>>& grid);
    std::optional<std::vector<Cell>> AStar(const std::vector<std::vector<bool>>& grid,
                                           int startCol, int startRow, int endCol,
                                           int endRow);
    std::vector<Cell> CompactPath(const std::vector<Cell>& path,
                                  const std::vector<std::vector<bool>>& grid);

    PathfinderOptions options_;
    int cols_{0};
    int rows_{0};
    std::vector<std::vector<bool>> obstacleGrid_;
    std::vector<ObstaclePolygon> forbiddenZones_;
};

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy

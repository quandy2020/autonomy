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

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "autonomy/map/strata/navigation/pathfinder.hpp"
#include "autonomy/map/strata/utils/point_util.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace navigation {

namespace {
constexpr int kNeighbors[8][2] = {{-1, -1}, {0, -1}, {1, -1}, {-1, 0},
                                  {1, 0},   {-1, 1}, {0, 1},  {1, 1}};
}

Pathfinder::Pathfinder(const PathfinderOptions& options) : options_(options) {
    if (options_.widthMeters <= 0 || options_.heightMeters <= 0) {
        throw std::invalid_argument("Pathfinder: widthMeters and heightMeters are required");
    }
    cols_ = static_cast<int>(std::ceil(options_.widthMeters / options_.gridSize));
    rows_ = static_cast<int>(std::ceil(options_.heightMeters / options_.gridSize));
}

void Pathfinder::SetObstacles(const std::vector<ObstaclePolygon>& obstacles) {
    forbiddenZones_ = obstacles;
    obstacleGrid_ = BuildObstacleGrid(obstacles);
}

std::optional<std::vector<std::pair<double, double>>> Pathfinder::FindPath(
    const std::pair<double, double>& start, const std::pair<double, double>& end) {
    if (obstacleGrid_.empty()) {
        return std::nullopt;
    }

    const int rawStartCol = std::min(cols_ - 1, std::max(0, static_cast<int>(std::round(start.first * cols_))));
    const int rawStartRow = std::min(rows_ - 1, std::max(0, static_cast<int>(std::round(start.second * rows_))));
    const int rawEndCol = std::min(cols_ - 1, std::max(0, static_cast<int>(std::round(end.first * cols_))));
    const int rawEndRow = std::min(rows_ - 1, std::max(0, static_cast<int>(std::round(end.second * rows_))));

    const auto startCell = NearestWalkableCell({rawStartCol, rawStartRow}, obstacleGrid_);
    const auto endCell = NearestWalkableCell({rawEndCol, rawEndRow}, obstacleGrid_);
    if (!startCell || !endCell) {
        return std::nullopt;
    }

    if (startCell->col == endCell->col && startCell->row == endCell->row) {
        return std::vector<std::pair<double, double>>{start};
    }

    const auto gridPath =
        AStar(obstacleGrid_, startCell->col, startCell->row, endCell->col, endCell->row);
    if (!gridPath) {
        return std::nullopt;
    }

    const auto compacted = CompactPath(*gridPath, obstacleGrid_);
    std::vector<std::pair<double, double>> result;
    result.reserve(compacted.size());
    for (const auto& cell : compacted) {
        result.emplace_back(static_cast<double>(cell.col) / cols_,
                            static_cast<double>(cell.row) / rows_);
    }
    return result;
}

std::vector<std::vector<bool>> Pathfinder::BuildObstacleGrid(
    const std::vector<ObstaclePolygon>& obstacles) {
    std::vector<std::vector<bool>> grid(rows_, std::vector<bool>(cols_, true));

    for (const auto& obstacle : obstacles) {
        if (obstacle.polygon.size() < 3) {
            continue;
        }
        double minX = std::numeric_limits<double>::infinity();
        double minY = std::numeric_limits<double>::infinity();
        double maxX = -std::numeric_limits<double>::infinity();
        double maxY = -std::numeric_limits<double>::infinity();
        for (const auto& pt : obstacle.polygon) {
            minX = std::min(minX, pt.x);
            minY = std::min(minY, pt.y);
            maxX = std::max(maxX, pt.x);
            maxY = std::max(maxY, pt.y);
        }

        const int minCol = std::max(0, static_cast<int>(std::floor(minX * cols_)));
        const int maxCol = std::min(cols_ - 1, static_cast<int>(std::ceil(maxX * cols_)));
        const int minRow = std::max(0, static_cast<int>(std::floor(minY * rows_)));
        const int maxRow = std::min(rows_ - 1, static_cast<int>(std::ceil(maxY * rows_)));

        for (int row = minRow; row <= maxRow; ++row) {
            for (int col = minCol; col <= maxCol; ++col) {
                const double fx = static_cast<double>(col) / cols_;
                const double fy = static_cast<double>(row) / rows_;
                if (utils::PointInPolygon(fx, fy, obstacle.polygon)) {
                    grid[row][col] = false;
                }
            }
        }
    }

    for (int pass = 0; pass < options_.dilatePasses; ++pass) {
        auto next = grid;
        for (int row = 0; row < rows_; ++row) {
            for (int col = 0; col < cols_; ++col) {
                if (!grid[row][col]) {
                    if (row > 0) next[row - 1][col] = false;
                    if (row < rows_ - 1) next[row + 1][col] = false;
                    if (col > 0) next[row][col - 1] = false;
                    if (col < cols_ - 1) next[row][col + 1] = false;
                }
            }
        }
        grid = std::move(next);
    }

    for (int row = 0; row < rows_; ++row) {
        for (int col = 0; col < cols_; ++col) {
            const double fx = static_cast<double>(col) / cols_;
            const double fy = static_cast<double>(row) / rows_;
            if (fx < options_.marginLeft || fx > 1.0 - options_.marginRight ||
                fy > 1.0 - options_.marginBottom || fy < options_.marginTop) {
                grid[row][col] = false;
            }
        }
    }
    return grid;
}

std::optional<Pathfinder::Cell> Pathfinder::NearestWalkableCell(
    const Cell& cell, const std::vector<std::vector<bool>>& grid) {
    if (grid[cell.row][cell.col]) {
        return cell;
    }
    std::queue<Cell> queue;
    std::unordered_set<int> visited;
    queue.push(cell);
    visited.insert(cell.row * cols_ + cell.col);

    while (!queue.empty()) {
        const Cell current = queue.front();
        queue.pop();
        if (grid[current.row][current.col]) {
            return current;
        }
        for (const auto& offset : kNeighbors) {
            const int ncol = current.col + offset[0];
            const int nrow = current.row + offset[1];
            if (ncol < 0 || ncol >= cols_ || nrow < 0 || nrow >= rows_) {
                continue;
            }
            const int idx = nrow * cols_ + ncol;
            if (visited.count(idx)) {
                continue;
            }
            visited.insert(idx);
            queue.push({ncol, nrow});
        }
    }
    return std::nullopt;
}

std::optional<std::vector<Pathfinder::Cell>> Pathfinder::AStar(
    const std::vector<std::vector<bool>>& grid, int startCol, int startRow, int endCol,
    int endRow) {
    struct Node {
        int col;
        int row;
        double f;
        bool operator>(const Node& other) const { return f > other.f; }
    };

    const auto index = [this](int col, int row) { return row * cols_ + col; };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<int, double> gScore;
    std::unordered_map<int, int> cameFrom;

    const int startIdx = index(startCol, startRow);
    const int endIdx = index(endCol, endRow);
    gScore[startIdx] = 0;
    open.push({startCol, startRow, std::hypot(endCol - startCol, endRow - startRow)});

    while (!open.empty()) {
        const Node current = open.top();
        open.pop();
        const int currentIdx = index(current.col, current.row);
        if (currentIdx == endIdx) {
            std::vector<Cell> path;
            int cursor = endIdx;
            while (cursor != startIdx) {
                path.push_back({cursor % cols_, cursor / cols_});
                cursor = cameFrom[cursor];
            }
            path.push_back({startCol, startRow});
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const auto& offset : kNeighbors) {
            const int ncol = current.col + offset[0];
            const int nrow = current.row + offset[1];
            if (ncol < 0 || ncol >= cols_ || nrow < 0 || nrow >= rows_ || !grid[nrow][ncol]) {
                continue;
            }
            const bool diagonal = offset[0] != 0 && offset[1] != 0;
            const double stepCost = diagonal ? (1.0 + options_.diagonalPenalty) : 1.0;
            const int neighborIdx = index(ncol, nrow);
            const double tentativeG = gScore[currentIdx] + stepCost;
            if (gScore.count(neighborIdx) && tentativeG >= gScore[neighborIdx]) {
                continue;
            }
            cameFrom[neighborIdx] = currentIdx;
            gScore[neighborIdx] = tentativeG;
            const double h = std::hypot(endCol - ncol, endRow - nrow);
            open.push({ncol, nrow, tentativeG + h});
        }
    }
    return std::nullopt;
}

std::vector<Pathfinder::Cell> Pathfinder::CompactPath(
    const std::vector<Cell>& path, const std::vector<std::vector<bool>>& grid) {
    if (path.size() <= 2) {
        return path;
    }
    std::vector<Cell> result;
    result.push_back(path.front());
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        const auto& prev = result.back();
        const auto& next = path[i + 1];
        const auto& current = path[i];
        const bool collinear =
            (next.col - prev.col) * (current.row - prev.row) ==
            (next.row - prev.row) * (current.col - prev.col);
        if (!collinear) {
            result.push_back(current);
        }
    }
    result.push_back(path.back());
    return result;
}

}  // namespace navigation
}  // namespace strata
}  // namespace map
}  // namespace autonomy

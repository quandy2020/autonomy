/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "navigation/planning/tasks/frontier_detector_task.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <utility>

#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace planning
{
namespace
{

int GridWidth(const automsgs::msgs::map_msgs::OccupancyGrid& grid)
{
    return static_cast<int>(grid.info().width());
}

int GridHeight(const automsgs::msgs::map_msgs::OccupancyGrid& grid)
{
    return static_cast<int>(grid.info().height());
}

std::int32_t At(const automsgs::msgs::map_msgs::OccupancyGrid& grid, int x, int y)
{
    if (x < 0 || y < 0 || x >= GridWidth(grid) || y >= GridHeight(grid))
    {
        return -1;
    }
    return grid.data(static_cast<int>(y) * GridWidth(grid) + x);
}

double OriginYaw(const automsgs::msgs::map_msgs::OccupancyGrid& grid)
{
    const auto& q = grid.info().origin().orientation();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

common::Pose2D CellToMap(const automsgs::msgs::map_msgs::OccupancyGrid& grid,
                         int x, int y)
{
    const double resolution = grid.info().resolution();
    const double local_x = (static_cast<double>(x) + 0.5) * resolution;
    const double local_y = (static_cast<double>(y) + 0.5) * resolution;
    const double yaw = OriginYaw(grid);
    const auto& origin = grid.info().origin().position();
    common::Pose2D pose;
    pose.set_x(origin.x() + std::cos(yaw) * local_x - std::sin(yaw) * local_y);
    pose.set_y(origin.y() + std::sin(yaw) * local_x + std::cos(yaw) * local_y);
    return pose;
}

bool IsFrontier(const automsgs::msgs::map_msgs::OccupancyGrid& grid, int x, int y)
{
    if (At(grid, x, y) != 0)
    {
        return false;
    }
    return At(grid, x + 1, y) < 0 || At(grid, x - 1, y) < 0 ||
           At(grid, x, y + 1) < 0 || At(grid, x, y - 1) < 0;
}

}  // namespace

std::vector<Waypoint> FrontierDetectorTask::Detect(
    const automsgs::msgs::map_msgs::OccupancyGrid& grid,
    const common::Pose2D& robot_pose, double max_range) const
{
    std::vector<Waypoint> waypoints;
    const int width = GridWidth(grid);
    const int height = GridHeight(grid);
    if (width <= 0 || height <= 0 ||
        grid.data_size() != width * height)
    {
        return waypoints;
    }
    std::vector<std::uint8_t> visited(static_cast<std::size_t>(width) * height, 0);
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const std::size_t index = static_cast<std::size_t>(y) * width + x;
            if (visited[index] != 0 || !IsFrontier(grid, x, y))
            {
                continue;
            }
            double sum_x = 0.0;
            double sum_y = 0.0;
            int count = 0;
            std::queue<std::pair<int, int>> queue;
            queue.emplace(x, y);
            visited[index] = 1;
            while (!queue.empty())
            {
                const auto [cx, cy] = queue.front();
                queue.pop();
                const auto pose = CellToMap(grid, cx, cy);
                sum_x += pose.x();
                sum_y += pose.y();
                ++count;
                const int neighbors[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                for (const auto& offset : neighbors)
                {
                    const int nx = cx + offset[0];
                    const int ny = cy + offset[1];
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                    {
                        continue;
                    }
                    const std::size_t next =
                        static_cast<std::size_t>(ny) * width + nx;
                    if (visited[next] == 0 && IsFrontier(grid, nx, ny))
                    {
                        visited[next] = 1;
                        queue.emplace(nx, ny);
                    }
                }
            }
            if (count == 0)
            {
                continue;
            }
            Waypoint waypoint;
            waypoint.pose.set_x(sum_x / count);
            waypoint.pose.set_y(sum_y / count);
            if (max_range > 0.0 &&
                common::Distance(waypoint.pose, robot_pose) > max_range)
            {
                continue;
            }
            waypoints.push_back(waypoint);
        }
    }
    std::sort(waypoints.begin(), waypoints.end(),
              [&robot_pose](const Waypoint& lhs, const Waypoint& rhs) {
                  return common::Distance(lhs.pose, robot_pose) <
                         common::Distance(rhs.pose, robot_pose);
              });
    return waypoints;
}

}  // namespace planning
}  // namespace navigation

/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/control/controller/teb_controller/tools/costmap_obstacle_converter.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {
namespace tools {

namespace {
using map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using map::costmap_2d::LETHAL_OBSTACLE;

double getYaw(const commsgs::geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

struct Cell {
  unsigned int mx{0};
  unsigned int my{0};
};

}  // namespace

CostmapObstacleConverter::CostmapObstacleConverter(
    const proto::TEBControllerOptions& options)
    : options_(options) {}

void CostmapObstacleConverter::update(
    const map::costmap_2d::Costmap2D& costmap,
    const commsgs::geometry_msgs::Pose& robot_pose) {
  obstacles_.clear();
  storage_.clear();

  if (!options_.include_costmap_obstacles()) {
    return;
  }

  const std::string mode = options_.obstacle_conversion_mode().empty()
                               ? "points_lines_polygons"
                               : options_.obstacle_conversion_mode();
  if (mode == "points_only") {
    UpdateAsPoints(costmap, robot_pose);
  } else {
    UpdateAsClusters(costmap, robot_pose);
  }
}

void CostmapObstacleConverter::UpdateAsPoints(
    const map::costmap_2d::Costmap2D& costmap,
    const commsgs::geometry_msgs::Pose& robot_pose) {
  const double resolution =
      options_.costmap_obstacle_sample_resolution() > 0.0
          ? options_.costmap_obstacle_sample_resolution()
          : costmap.getResolution();
  const int stride = std::max(
      1, static_cast<int>(std::lround(resolution / costmap.getResolution())));

  const double yaw = getYaw(robot_pose.orientation);
  const double cos_th = std::cos(yaw);
  const double sin_th = std::sin(yaw);
  const double behind_dist =
      options_.costmap_obstacles_behind_robot_dist() > 0.0
          ? options_.costmap_obstacles_behind_robot_dist()
          : 0.0;

  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();

  for (unsigned int my = 0; my < size_y; my += static_cast<unsigned int>(stride)) {
    for (unsigned int mx = 0; mx < size_x;
         mx += static_cast<unsigned int>(stride)) {
      const unsigned char cost = costmap.getCost(mx, my);
      if (cost < INSCRIBED_INFLATED_OBSTACLE) {
        continue;
      }

      double wx = 0.0;
      double wy = 0.0;
      costmap.mapToWorld(mx, my, wx, wy);

      const double dx = wx - robot_pose.position.x;
      const double dy = wy - robot_pose.position.y;
      const double forward = dx * cos_th + dy * sin_th;
      if (forward < -behind_dist) {
        continue;
      }

      auto obstacle = std::make_shared<PointObstacle>(wx, wy);
      storage_.push_back(obstacle);
      obstacles_.push_back(obstacle);
    }
  }
}

void CostmapObstacleConverter::UpdateAsClusters(
    const map::costmap_2d::Costmap2D& costmap,
    const commsgs::geometry_msgs::Pose& robot_pose) {
  const double yaw = getYaw(robot_pose.orientation);
  const double cos_th = std::cos(yaw);
  const double sin_th = std::sin(yaw);
  const double behind_dist =
      options_.costmap_obstacles_behind_robot_dist() > 0.0
          ? options_.costmap_obstacles_behind_robot_dist()
          : 0.0;

  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  std::vector<uint8_t> visited(size_x * size_y, 0);

  auto index = [size_x](unsigned int mx, unsigned int my) {
    return my * size_x + mx;
  };
  auto is_obstacle = [&](unsigned int mx, unsigned int my) {
    return costmap.getCost(mx, my) >= INSCRIBED_INFLATED_OBSTACLE;
  };
  auto in_front = [&](double wx, double wy) {
    const double dx = wx - robot_pose.position.x;
    const double dy = wy - robot_pose.position.y;
    return (dx * cos_th + dy * sin_th) >= -behind_dist;
  };

  const int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

  for (unsigned int seed_y = 0; seed_y < size_y; ++seed_y) {
    for (unsigned int seed_x = 0; seed_x < size_x; ++seed_x) {
      if (visited[index(seed_x, seed_y)] || !is_obstacle(seed_x, seed_y)) {
        continue;
      }

      std::vector<Cell> cluster;
      std::queue<Cell> q;
      q.push({seed_x, seed_y});
      visited[index(seed_x, seed_y)] = 1;

      while (!q.empty()) {
        const Cell cur = q.front();
        q.pop();
        double wx = 0.0;
        double wy = 0.0;
        costmap.mapToWorld(cur.mx, cur.my, wx, wy);
        if (in_front(wx, wy)) {
          cluster.push_back(cur);
        }
        for (const auto& d : dirs) {
          const int nx = static_cast<int>(cur.mx) + d[0];
          const int ny = static_cast<int>(cur.my) + d[1];
          if (nx < 0 || ny < 0 ||
              static_cast<unsigned int>(nx) >= size_x ||
              static_cast<unsigned int>(ny) >= size_y) {
            continue;
          }
          const unsigned int umx = static_cast<unsigned int>(nx);
          const unsigned int umy = static_cast<unsigned int>(ny);
          if (visited[index(umx, umy)] || !is_obstacle(umx, umy)) {
            continue;
          }
          visited[index(umx, umy)] = 1;
          q.push({umx, umy});
        }
      }

      if (cluster.empty()) {
        continue;
      }

      std::vector<Eigen::Vector2d> pts;
      pts.reserve(cluster.size());
      double cx = 0.0;
      double cy = 0.0;
      for (const auto& c : cluster) {
        double wx = 0.0;
        double wy = 0.0;
        costmap.mapToWorld(c.mx, c.my, wx, wy);
        pts.emplace_back(wx, wy);
        cx += wx;
        cy += wy;
      }
      cx /= static_cast<double>(pts.size());
      cy /= static_cast<double>(pts.size());

      if (pts.size() <= 2) {
        auto obstacle = std::make_shared<PointObstacle>(cx, cy);
        storage_.push_back(obstacle);
        obstacles_.push_back(obstacle);
        continue;
      }

      // PCA aspect ratio for line vs polygon.
      double cxx = 0.0;
      double cyy = 0.0;
      double cxy = 0.0;
      for (const auto& p : pts) {
        const double dx = p.x() - cx;
        const double dy = p.y() - cy;
        cxx += dx * dx;
        cyy += dy * dy;
        cxy += dx * dy;
      }
      const double n = static_cast<double>(pts.size());
      cxx /= n;
      cyy /= n;
      cxy /= n;
      const double trace = cxx + cyy;
      const double det = cxx * cyy - cxy * cxy;
      const double disc = std::max(0.0, trace * trace * 0.25 - det);
      const double lambda1 = trace * 0.5 + std::sqrt(disc);
      const double lambda2 = trace * 0.5 - std::sqrt(disc);
      const double ratio =
          (lambda2 > 1e-9) ? (lambda1 / lambda2) : 1e9;

      if (ratio > 4.0) {
        Eigen::Vector2d axis(1.0, 0.0);
        if (std::abs(cxy) > 1e-12 || std::abs(cxx - lambda1) > 1e-12) {
          axis = Eigen::Vector2d(lambda1 - cyy, cxy);
          if (axis.squaredNorm() < 1e-12) {
            axis = Eigen::Vector2d(cxy, lambda1 - cxx);
          }
        }
        axis.normalize();
        double min_proj = 0.0;
        double max_proj = 0.0;
        for (size_t i = 0; i < pts.size(); ++i) {
          const double proj = (pts[i] - Eigen::Vector2d(cx, cy)).dot(axis);
          if (i == 0 || proj < min_proj) {
            min_proj = proj;
          }
          if (i == 0 || proj > max_proj) {
            max_proj = proj;
          }
        }
        const Eigen::Vector2d p0 = Eigen::Vector2d(cx, cy) + min_proj * axis;
        const Eigen::Vector2d p1 = Eigen::Vector2d(cx, cy) + max_proj * axis;
        auto obstacle = std::make_shared<LineObstacle>(p0, p1);
        storage_.push_back(obstacle);
        obstacles_.push_back(obstacle);
        continue;
      }

      // Axis-aligned bounding box as a 4-vertex polygon.
      double min_x = pts.front().x();
      double max_x = pts.front().x();
      double min_y = pts.front().y();
      double max_y = pts.front().y();
      for (const auto& p : pts) {
        min_x = std::min(min_x, p.x());
        max_x = std::max(max_x, p.x());
        min_y = std::min(min_y, p.y());
        max_y = std::max(max_y, p.y());
      }
      Point2dContainer vertices;
      vertices.emplace_back(min_x, min_y);
      vertices.emplace_back(max_x, min_y);
      vertices.emplace_back(max_x, max_y);
      vertices.emplace_back(min_x, max_y);
      auto obstacle = std::make_shared<PolygonObstacle>(vertices);
      storage_.push_back(obstacle);
      obstacles_.push_back(obstacle);
    }
  }
}

}  // namespace tools
}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

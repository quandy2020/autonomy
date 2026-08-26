/*
 * helpers.hpp
 *
 *  Created on: Nov 20, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 *
 * Adapted for autonomy (ROS-free).
 */

#pragma once

#include <chrono>
#include <string>

#include <pcl/common/common.h>
#include <Eigen/Geometry>

namespace grid_map {

class GridMapPclLoader;
class GridMap;

namespace grid_map_pcl {

using Point = ::pcl::PointXYZ;
using Pointcloud = ::pcl::PointCloud<Point>;
enum class XYZ : int { X, Y, Z };

void printTimeElapsed(const std::chrono::system_clock::time_point& start,
                      const std::string& prefix);

/*!
 * Preprocess the cloud, initialize geometry and add an elevation layer.
 * @param gridMapPclLoader loader instance
 * @param mapLayerName layer name to populate (default: elevation)
 */
void processPointcloud(grid_map::GridMapPclLoader* gridMapPclLoader,
                      const std::string& mapLayerName = "elevation");

Eigen::Affine3f getRigidBodyTransform(const Eigen::Vector3d& translation,
                                      const Eigen::Vector3d& intrinsicRpy);
Eigen::Matrix3f getRotationMatrix(double angle, XYZ axis);

Eigen::Vector3d calculateMeanOfPointPositions(Pointcloud::ConstPtr inputCloud);
Pointcloud::Ptr transformCloud(Pointcloud::ConstPtr inputCloud,
                               const Eigen::Affine3f& transformMatrix);
Pointcloud::Ptr loadPointcloudFromPcd(const std::string& filename);

}  // namespace grid_map_pcl
}  // namespace grid_map

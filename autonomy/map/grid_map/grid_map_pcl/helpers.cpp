/*
 * helpers.cpp
 *
 *  Created on: Nov 20, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 *
 * Adapted for autonomy (ROS-free).
 */

#include "autonomy/map/grid_map/grid_map_pcl/helpers.hpp"

#include <memory>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_pcl/grid_map_pcl_loader.hpp"

namespace grid_map {
namespace grid_map_pcl {

void printTimeElapsed(const std::chrono::system_clock::time_point& start,
                      const std::string& prefix) {
  const auto stop = std::chrono::high_resolution_clock::now();
  const auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() /
      1000.0;
  AINFO << prefix << duration << " sec";
}

void processPointcloud(grid_map::GridMapPclLoader* gridMapPclLoader,
                      const std::string& mapLayerName) {
  const auto start = std::chrono::high_resolution_clock::now();
  gridMapPclLoader->preProcessInputCloud();
  gridMapPclLoader->initializeGridMapGeometryFromInputCloud();
  printTimeElapsed(start, "Initialization took: ");
  gridMapPclLoader->addLayerFromInputCloud(mapLayerName);
  printTimeElapsed(start, "Total time: ");
}

Eigen::Affine3f getRigidBodyTransform(const Eigen::Vector3d& translation,
                                      const Eigen::Vector3d& intrinsicRpy) {
  Eigen::Affine3f rigidBodyTransform;
  rigidBodyTransform.setIdentity();
  rigidBodyTransform.translation() << translation.x(), translation.y(),
      translation.z();
  Eigen::Matrix3f rotation(Eigen::Matrix3f::Identity());
  rotation *= getRotationMatrix(intrinsicRpy.x(), XYZ::X);
  rotation *= getRotationMatrix(intrinsicRpy.y(), XYZ::Y);
  rotation *= getRotationMatrix(intrinsicRpy.z(), XYZ::Z);
  rigidBodyTransform.rotate(rotation);
  return rigidBodyTransform;
}

Eigen::Matrix3f getRotationMatrix(double angle, XYZ axis) {
  Eigen::Matrix3f rotationMatrix = Eigen::Matrix3f::Identity();
  switch (axis) {
    case XYZ::X:
      rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX());
      break;
    case XYZ::Y:
      rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY());
      break;
    case XYZ::Z:
      rotationMatrix = Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ());
      break;
    default:
      AERROR << "Unknown axis while trying to rotate the pointcloud";
  }
  return rotationMatrix;
}

Eigen::Vector3d calculateMeanOfPointPositions(Pointcloud::ConstPtr inputCloud) {
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& point : inputCloud->points) {
    mean += Eigen::Vector3d(point.x, point.y, point.z);
  }
  mean /= inputCloud->points.size();
  return mean;
}

Pointcloud::Ptr loadPointcloudFromPcd(const std::string& filename) {
  Pointcloud::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloudBlob;
  pcl::io::loadPCDFile(filename, cloudBlob);
  pcl::fromPCLPointCloud2(cloudBlob, *cloud);
  return cloud;
}

Pointcloud::Ptr transformCloud(Pointcloud::ConstPtr inputCloud,
                               const Eigen::Affine3f& transformMatrix) {
  Pointcloud::Ptr transformedCloud(new Pointcloud());
  pcl::transformPointCloud(*inputCloud, *transformedCloud, transformMatrix);
  return transformedCloud;
}

}  // namespace grid_map_pcl
}  // namespace grid_map

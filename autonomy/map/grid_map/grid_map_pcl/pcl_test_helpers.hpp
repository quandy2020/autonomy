/*
 * pcl_test_helpers.hpp
 */

#pragma once

#include <random>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <Eigen/Core>

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_core/iterators/grid_map_iterator.hpp"

namespace grid_map {

class GridMapPclLoader;

namespace grid_map_pcl_test {

using Point = pcl::PointXYZ;
using Pointcloud = pcl::PointCloud<Point>;

static const std::string layerName = "elevation";
static const bool savePointclouds = false;
extern std::mt19937 rndGenerator;

std::string getConfigFilePath();
std::string getTestDataFolderPath();
std::string getTestPcdFilePath();

Pointcloud::Ptr concatenate(Pointcloud::Ptr cloud1, Pointcloud::Ptr cloud2);
std::vector<double> getNonNanElevationValues(const grid_map::GridMap& gridMap);
std::vector<Eigen::Vector3d> getNonNanElevationValuesWithCoordinates(
    const grid_map::GridMap& gridMap);

Pointcloud::Ptr createNormallyDistributedBlobOfPoints(unsigned int nPoints,
                                                      double mean, double stdDev,
                                                      std::mt19937* generator);
Pointcloud::Ptr createNoisyPlanePointcloud(unsigned int nPoints, double minXY,
                                          double maxXY, double meanZ,
                                          double stdDevZ, std::mt19937* generator);
Pointcloud::Ptr makePerfectPlane(unsigned int nPoints, double minXY, double maxXY,
                                 double desiredHeight, std::mt19937* generator);
Pointcloud::Ptr createStepTerrain(unsigned int nPoints, double minXY,
                                  double maxXY, double zHigh, double zLow,
                                  double stdDevZ, std::mt19937* generator,
                                  double* center);

void runGridMapPclLoaderOnInputCloud(Pointcloud::ConstPtr inputCloud,
                                     grid_map::GridMapPclLoader* gridMapPclLoader);

class PointcloudCreator {
 public:
  static Pointcloud::Ptr createNoisyPointcloudOfStepTerrain(double* stepLocation,
                                                           double* zHigh,
                                                           double* zLow,
                                                           double* stdDev);
  static Pointcloud::Ptr createBlobOfPoints(double* mean, double* stdDev);
  static Pointcloud::Ptr createVerticesOfASquare(double* x, double* y);
  static Pointcloud::Ptr createNoisyPlane(double* height, double* stdDevZ);
  static Pointcloud::Ptr createNoisyDoublePlane(double* minZ, double* stdDevZ);
  static Pointcloud::Ptr createPerfectPlane(double* height);
  static Pointcloud::Ptr createNBlobsAboveEachOther(double* minZ, double* stdDevZ,
                                                   int* nBlobs);
};

}  // namespace grid_map_pcl_test
}  // namespace grid_map

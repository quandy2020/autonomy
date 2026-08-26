/*
 * pcl_test_helpers.cpp
 */

#include "autonomy/map/grid_map/grid_map_pcl/pcl_test_helpers.hpp"

#include <filesystem>

#include "autonomy/map/grid_map/grid_map_pcl/grid_map_pcl_loader.hpp"

namespace grid_map {
namespace grid_map_pcl_test {

std::mt19937 rndGenerator;

std::string getTestDataFolderPath() {
  return (std::filesystem::path(__FILE__).parent_path() / "test_data").string();
}

std::string getConfigFilePath() {
  return getTestDataFolderPath() + "/parameters.yaml";
}

std::string getTestPcdFilePath() {
  return getTestDataFolderPath() + "/plane_noisy.pcd";
}

std::vector<Eigen::Vector3d> getNonNanElevationValuesWithCoordinates(
    const grid_map::GridMap& gridMap) {
  std::vector<Eigen::Vector3d> nonNanCoordinates;
  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd();
       ++iterator) {
    double value = gridMap.at(layerName, *iterator);
    if (!std::isnan(value)) {
      grid_map::Position position;
      gridMap.getPosition(grid_map::Index(*iterator), position);
      nonNanCoordinates.emplace_back(position.x(), position.y(), value);
    }
  }
  return nonNanCoordinates;
}

std::vector<double> getNonNanElevationValues(const grid_map::GridMap& gridMap) {
  std::vector<double> nonNanElevations;
  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd();
       ++iterator) {
    double value = gridMap.at(layerName, *iterator);
    if (!std::isnan(value)) {
      nonNanElevations.push_back(value);
    }
  }
  return nonNanElevations;
}

Pointcloud::Ptr createNormallyDistributedBlobOfPoints(unsigned int nPoints,
                                                      double mean, double stdDev,
                                                      std::mt19937* generator) {
  std::normal_distribution<double> normalDist(mean, stdDev);
  Pointcloud::Ptr cloud(new Pointcloud());
  cloud->points.reserve(nPoints);
  for (unsigned int i = 0; i < nPoints; ++i) {
    Point point;
    point.x = normalDist(*generator);
    point.y = normalDist(*generator);
    point.z = normalDist(*generator);
    cloud->push_back(point);
  }
  return cloud;
}

Pointcloud::Ptr createNoisyPlanePointcloud(unsigned int nPoints, double minXY,
                                          double maxXY, double meanZ,
                                          double stdDevZ,
                                          std::mt19937* generator) {
  std::uniform_real_distribution<double> uniformDist(minXY, maxXY);
  std::normal_distribution<double> normalDist(meanZ, stdDevZ);
  Pointcloud::Ptr cloud(new Pointcloud());
  cloud->points.reserve(nPoints);
  for (unsigned int i = 0; i < nPoints; ++i) {
    Point point;
    point.x = uniformDist(*generator);
    point.y = uniformDist(*generator);
    point.z = normalDist(*generator);
    cloud->push_back(point);
  }
  return cloud;
}

Pointcloud::Ptr makePerfectPlane(unsigned int nPoints, double minXY, double maxXY,
                                 double desiredHeight, std::mt19937* generator) {
  std::uniform_real_distribution<double> uniformDist(minXY, maxXY);
  Pointcloud::Ptr cloud(new Pointcloud());
  cloud->points.reserve(nPoints);
  for (unsigned int i = 0; i < nPoints; ++i) {
    Point point;
    point.x = uniformDist(*generator);
    point.y = uniformDist(*generator);
    point.z = desiredHeight;
    cloud->push_back(point);
  }
  return cloud;
}

Pointcloud::Ptr concatenate(Pointcloud::Ptr cloud1, Pointcloud::Ptr cloud2) {
  Pointcloud::Ptr concatenatedCloud(new Pointcloud());
  concatenatedCloud->points.reserve(cloud1->points.size() +
                                    cloud2->points.size());
  std::copy(cloud2->points.begin(), cloud2->points.end(),
            std::back_inserter(concatenatedCloud->points));
  std::copy(cloud1->points.begin(), cloud1->points.end(),
            std::back_inserter(concatenatedCloud->points));
  return concatenatedCloud;
}

void runGridMapPclLoaderOnInputCloud(
    Pointcloud::ConstPtr inputCloud,
    grid_map::GridMapPclLoader* gridMapPclLoader) {
  gridMapPclLoader->loadParameters(getConfigFilePath());
  gridMapPclLoader->setInputCloud(inputCloud);
  gridMapPclLoader->preProcessInputCloud();
  gridMapPclLoader->initializeGridMapGeometryFromInputCloud();
  gridMapPclLoader->addLayerFromInputCloud(layerName);
}

Pointcloud::Ptr createStepTerrain(unsigned int nPoints, double minXY,
                                  double maxXY, double zHigh, double zLow,
                                  double stdDevZ, std::mt19937* generator,
                                  double* center) {
  *center = (maxXY + minXY) / 2.0;
  std::uniform_real_distribution<double> uniformDist(minXY, maxXY);
  std::normal_distribution<double> zLowDist(zLow, stdDevZ);
  std::normal_distribution<double> zHighDist(zHigh, stdDevZ);
  Pointcloud::Ptr cloud(new Pointcloud());
  cloud->points.reserve(nPoints);
  for (unsigned int i = 0; i < nPoints; ++i) {
    Point point;
    point.x = uniformDist(*generator);
    point.y = uniformDist(*generator);
    point.z = (point.x > *center) ? zHighDist(*generator) : zLowDist(*generator);
    cloud->push_back(point);
  }
  return cloud;
}

Pointcloud::Ptr PointcloudCreator::createNoisyPointcloudOfStepTerrain(
    double* stepLocationX, double* zHigh, double* zLow, double* stdDev) {
  std::uniform_real_distribution<double> heightDist(-2.0, 2.0);
  *stdDev = 0.01;
  *zHigh = heightDist(rndGenerator) + 2.1;
  *zLow = heightDist(rndGenerator) - 2.1;
  return createStepTerrain(200000, -3.0, 3.0, *zHigh, *zLow, *stdDev,
                           &rndGenerator, stepLocationX);
}

Pointcloud::Ptr PointcloudCreator::createBlobOfPoints(double* mean,
                                                     double* stdDev) {
  std::uniform_real_distribution<double> meanDist(-10.0, 10.0);
  std::uniform_real_distribution<double> sigmaDist(0.001, 0.1);
  *mean = meanDist(rndGenerator);
  *stdDev = sigmaDist(rndGenerator);
  return createNormallyDistributedBlobOfPoints(10000, *mean, *stdDev,
                                               &rndGenerator);
}

Pointcloud::Ptr PointcloudCreator::createVerticesOfASquare(double* x, double* y) {
  Pointcloud::Ptr cloud(new Pointcloud());
  std::uniform_real_distribution<double> zDist(-10.0, 10.0);
  std::uniform_int_distribution<int> xDist(10, 20);
  std::uniform_int_distribution<int> yDist(25, 40);
  *x = xDist(rndGenerator);
  *y = yDist(rndGenerator);
  cloud->points.emplace_back(*x, 0.0, zDist(rndGenerator));
  cloud->points.emplace_back(-(*x), 0.0, zDist(rndGenerator));
  cloud->points.emplace_back(0.0, *y, zDist(rndGenerator));
  cloud->points.emplace_back(0.0, -(*y), zDist(rndGenerator));
  cloud->is_dense = true;
  return cloud;
}

Pointcloud::Ptr PointcloudCreator::createNoisyDoublePlane(double* minZ,
                                                         double* stdDevZ) {
  std::uniform_real_distribution<double> upperPlaneZDist(0.0, 10.0);
  std::uniform_real_distribution<double> lowerPlaneZDist(-10.0, -5.0);
  std::uniform_real_distribution<double> stdDevDist(0.001, 0.1);
  *stdDevZ = stdDevDist(rndGenerator);
  *minZ = lowerPlaneZDist(rndGenerator);
  // Keep cloud size moderate for CI time.
  auto cloudLower = createNoisyPlanePointcloud(150000, -1.0, 1.0, *minZ, *stdDevZ,
                                              &rndGenerator);
  auto cloudUpper = createNoisyPlanePointcloud(
      150000, -1.0, 1.0, upperPlaneZDist(rndGenerator), *stdDevZ, &rndGenerator);
  return concatenate(cloudUpper, cloudLower);
}

Pointcloud::Ptr PointcloudCreator::createNoisyPlane(double* height,
                                                   double* stdDevZ) {
  std::uniform_real_distribution<double> heightDist(-10.0, 10.0);
  std::uniform_real_distribution<double> stdDevDist(0.001, 0.1);
  *stdDevZ = stdDevDist(rndGenerator);
  *height = heightDist(rndGenerator);
  return createNoisyPlanePointcloud(150000, -1.0, 1.0, *height, *stdDevZ,
                                    &rndGenerator);
}

Pointcloud::Ptr PointcloudCreator::createPerfectPlane(double* height) {
  std::uniform_real_distribution<double> heightDist(-10.0, 10.0);
  *height = heightDist(rndGenerator);
  return makePerfectPlane(80000, -3.0, 3.0, *height, &rndGenerator);
}

Pointcloud::Ptr PointcloudCreator::createNBlobsAboveEachOther(double* minZ,
                                                             double* stdDevZ,
                                                             int* nBlobs) {
  std::uniform_real_distribution<double> sigmaDist(0.001, 0.015);
  std::uniform_real_distribution<double> minZDist(-10.0, 10.0);
  std::uniform_int_distribution<int> nDist(10, 20);
  *nBlobs = nDist(rndGenerator);
  *minZ = minZDist(rndGenerator);
  *stdDevZ = sigmaDist(rndGenerator);
  Pointcloud::Ptr cloud(new Pointcloud());
  for (int i = 0; i < *nBlobs; ++i) {
    auto blob = createNormallyDistributedBlobOfPoints(
        1000, *minZ + i * 2.0, *stdDevZ, &rndGenerator);
    cloud = concatenate(cloud, blob);
  }
  return cloud;
}

}  // namespace grid_map_pcl_test
}  // namespace grid_map

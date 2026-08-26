/*
 * grid_map_pcl_loader_test.cpp
 */

#include <algorithm>
#include <cstdlib>
#include <iostream>

#include <gtest/gtest.h>

#include "autonomy/map/grid_map/grid_map_pcl/grid_map_pcl_loader.hpp"
#include "autonomy/map/grid_map/grid_map_pcl/pcl_test_helpers.hpp"

namespace grid_map {
namespace grid_map_pcl_test {

TEST(GridMapPclLoaderTest, FlatGroundRealDataset) {
  GridMapPclLoader gridMapPclLoader;
  gridMapPclLoader.loadParameters(getConfigFilePath());
  gridMapPclLoader.loadCloudFromPcdFile(getTestPcdFilePath());
  gridMapPclLoader.preProcessInputCloud();
  gridMapPclLoader.initializeGridMapGeometryFromInputCloud();
  gridMapPclLoader.addLayerFromInputCloud(layerName);

  const auto& gridMap = gridMapPclLoader.getGridMap();
  const auto elevationValues = getNonNanElevationValues(gridMap);

  EXPECT_TRUE(!elevationValues.empty());

  const double referenceElevation = elevationValues.front();
  for (const auto& elevation : elevationValues) {
    EXPECT_NEAR(elevation, referenceElevation, 3e-2);
  }
}

TEST(GridMapPclLoaderTest, PerfectPlane) {
  const auto seed = static_cast<unsigned int>(rand());
  rndGenerator.seed(seed);

  const int numTests = 5;
  for (int i = 0; i < numTests; ++i) {
    double height = 0.0;
    auto cloud = PointcloudCreator::createPerfectPlane(&height);
    GridMapPclLoader gridMapPclLoader;
    runGridMapPclLoaderOnInputCloud(cloud, &gridMapPclLoader);
    const auto elevationValues =
        getNonNanElevationValues(gridMapPclLoader.getGridMap());

    EXPECT_TRUE(!elevationValues.empty());
    for (const auto& elevation : elevationValues) {
      EXPECT_NEAR(elevation, height, 1e-5);
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test PerfectPlane failed with seed: " << seed << std::endl;
  }
}

TEST(GridMapPclLoaderTest, NoisyPlane) {
  const auto seed = static_cast<unsigned int>(rand());
  rndGenerator.seed(seed);

  const int numTests = 3;
  for (int i = 0; i < numTests; ++i) {
    double height = 0.0;
    double stdDevZ = 0.0;
    auto cloud = PointcloudCreator::createNoisyPlane(&height, &stdDevZ);
    GridMapPclLoader gridMapPclLoader;
    runGridMapPclLoaderOnInputCloud(cloud, &gridMapPclLoader);
    const auto elevationValues =
        getNonNanElevationValues(gridMapPclLoader.getGridMap());

    EXPECT_TRUE(!elevationValues.empty());
    for (const auto& elevation : elevationValues) {
      EXPECT_NEAR(elevation, height, 3 * stdDevZ);
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test NoisyPlane failed with seed: " << seed << std::endl;
  }
}

TEST(GridMapPclLoaderTest, NoisyDoublePlane) {
  const auto seed = static_cast<unsigned int>(rand());
  rndGenerator.seed(seed);

  double minZ = 0.0;
  double stdDevZ = 0.0;
  auto cloud = PointcloudCreator::createNoisyDoublePlane(&minZ, &stdDevZ);
  GridMapPclLoader gridMapPclLoader;
  runGridMapPclLoaderOnInputCloud(cloud, &gridMapPclLoader);
  const auto elevationValues =
      getNonNanElevationValues(gridMapPclLoader.getGridMap());

  EXPECT_TRUE(!elevationValues.empty());
  for (const auto& elevation : elevationValues) {
    EXPECT_NEAR(elevation, minZ, 3 * stdDevZ);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test NoisyDoublePlane failed with seed: " << seed
              << std::endl;
  }
}

TEST(GridMapPclLoaderTest, InitializeGeometry) {
  const auto seed = static_cast<unsigned int>(rand());
  rndGenerator.seed(seed);

  const unsigned int numTests = 50;
  for (unsigned int i = 0; i < numTests; ++i) {
    double xLocation = 0.0;
    double yLocation = 0.0;
    auto cloud = PointcloudCreator::createVerticesOfASquare(&xLocation, &yLocation);
    GridMapPclLoader gridMapPclLoader;
    gridMapPclLoader.loadParameters(getConfigFilePath());
    gridMapPclLoader.setInputCloud(cloud);
    gridMapPclLoader.initializeGridMapGeometryFromInputCloud();
    auto gridMap = gridMapPclLoader.getGridMap();
    auto center = gridMap.getPosition();

    EXPECT_NEAR(center.x(), 0.0, 1e-5);
    EXPECT_NEAR(center.y(), 0.0, 1e-5);
    auto length = gridMap.getLength();
    EXPECT_NEAR(length.x(), std::round(2 * xLocation), 1e-5);
    EXPECT_NEAR(length.y(), std::round(2 * yLocation), 1e-5);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test InitializeGeometry failed with seed: " << seed
              << std::endl;
  }
}

TEST(GridMapPclLoaderTest, NoisyStepTerrain) {
  const auto seed = static_cast<unsigned int>(rand());
  rndGenerator.seed(seed);

  const int numTests = 3;
  for (int i = 0; i < numTests; ++i) {
    double stepLocationX = 0.0;
    double zHigh = 0.0;
    double zLow = 0.0;
    double stdDevZ = 0.0;
    const auto cloud = PointcloudCreator::createNoisyPointcloudOfStepTerrain(
        &stepLocationX, &zHigh, &zLow, &stdDevZ);
    GridMapPclLoader gridMapPclLoader;
    runGridMapPclLoaderOnInputCloud(cloud, &gridMapPclLoader);
    const auto coordinates =
        getNonNanElevationValuesWithCoordinates(gridMapPclLoader.getGridMap());

    EXPECT_TRUE(!coordinates.empty());
    for (const auto& coordinate : coordinates) {
      if (coordinate.x() > stepLocationX) {
        EXPECT_NEAR(coordinate.z(), zHigh, 3 * stdDevZ);
      }
      if (coordinate.x() < stepLocationX) {
        EXPECT_NEAR(coordinate.z(), zLow, 3 * stdDevZ);
      }
    }
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test NoisyStepTerrain failed with seed: " << seed
              << std::endl;
  }
}

TEST(GridMapPclLoaderTest, CalculateElevation) {
  const auto seed = static_cast<unsigned int>(rand());
  rndGenerator.seed(seed);

  const unsigned int numTests = 5;
  for (unsigned int i = 0; i < numTests; ++i) {
    double minZ = 0.0;
    double stdDevZ = 0.0;
    int nBlobs = 0;
    auto cloud =
        PointcloudCreator::createNBlobsAboveEachOther(&minZ, &stdDevZ, &nBlobs);
    GridMapPclLoader gridMapPclLoader;
    gridMapPclLoader.loadParameters(getConfigFilePath());
    gridMapPclLoader.setInputCloud(cloud);

    std::vector<float> clusterHeights;
    gridMapPclLoader.calculateElevationFromPointsInsideGridMapCell(
        cloud, clusterHeights);
    ASSERT_FALSE(clusterHeights.empty());
    const float elevation =
        *std::min_element(clusterHeights.begin(), clusterHeights.end());
    EXPECT_NEAR(elevation, minZ, 3 * stdDevZ);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test CalculateElevation failed with seed: " << seed
              << std::endl;
  }
}

TEST(GridMapPclLoaderTest, SavePointclouds) {
  if (!savePointclouds) {
    return;
  }

  const auto seed = static_cast<unsigned int>(rand());
  rndGenerator.seed(seed);

  double dummyDouble1 = 0.0;
  double dummyDouble2 = 0.0;
  double dummyDouble3 = 0.0;
  double dummyDouble4 = 0.0;
  int dummyInt = 0;

  GridMapPclLoader gridMapPclLoader;
  gridMapPclLoader.loadParameters(getConfigFilePath());

  std::string filename = getTestDataFolderPath() + "/perfectPlane.pcd";
  auto cloud = PointcloudCreator::createPerfectPlane(&dummyDouble1);
  gridMapPclLoader.setInputCloud(cloud);
  gridMapPclLoader.savePointCloudAsPcdFile(filename);

  filename = getTestDataFolderPath() + "/Nblobs.pcd";
  cloud = PointcloudCreator::createNBlobsAboveEachOther(&dummyDouble1,
                                                        &dummyDouble2, &dummyInt);
  gridMapPclLoader.setInputCloud(cloud);
  gridMapPclLoader.savePointCloudAsPcdFile(filename);
}

}  // namespace grid_map_pcl_test
}  // namespace grid_map

/*
 * pointcloud_processor_test.cpp
 */

#include <cstdlib>

#include <gtest/gtest.h>

#include "autonomy/map/grid_map/grid_map_pcl/pcl_test_helpers.hpp"
#include "autonomy/map/grid_map/grid_map_pcl/pointcloud_processor.hpp"

namespace grid_map {
namespace grid_map_pcl_test {

TEST(PointcloudProcessorTest, ExtractClusters) {
  const auto seed = static_cast<unsigned int>(rand());
  rndGenerator.seed(seed);

  const unsigned int numTests = 5;
  for (unsigned int i = 0; i < numTests; ++i) {
    double minZ = 0.0;
    double stdDevZ = 0.0;
    int nBlobs = 0;
    auto cloud =
        PointcloudCreator::createNBlobsAboveEachOther(&minZ, &stdDevZ, &nBlobs);
    grid_map_pcl::PointcloudProcessor pointcloudProcessor;
    pointcloudProcessor.loadParameters(getConfigFilePath());
    auto clusters =
        pointcloudProcessor.extractClusterCloudsFromPointcloud(cloud);
    EXPECT_EQ(static_cast<int>(clusters.size()), nBlobs);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test ExtractClusters failed with seed: " << seed
              << std::endl;
  }
}

}  // namespace grid_map_pcl_test
}  // namespace grid_map

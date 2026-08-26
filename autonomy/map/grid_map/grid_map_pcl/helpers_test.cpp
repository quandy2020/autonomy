/*
 * helpers_test.cpp
 */

#include <cstdlib>

#include <gtest/gtest.h>

#include "autonomy/map/grid_map/grid_map_pcl/helpers.hpp"
#include "autonomy/map/grid_map/grid_map_pcl/pcl_test_helpers.hpp"

namespace grid_map {
namespace grid_map_pcl_test {

TEST(HelpersTest, MeanPositionTest) {
  const auto seed = static_cast<unsigned int>(rand());
  rndGenerator.seed(seed);

  const unsigned int numTests = 10;
  for (unsigned int i = 0; i < numTests; ++i) {
    double mean = 0.0;
    double stdDev = 0.0;
    auto cloud = PointcloudCreator::createBlobOfPoints(&mean, &stdDev);
    auto meanOfAllPoints = grid_map_pcl::calculateMeanOfPointPositions(cloud);

    EXPECT_NEAR(meanOfAllPoints.x(), mean, 3 * stdDev);
    EXPECT_NEAR(meanOfAllPoints.y(), mean, 3 * stdDev);
    EXPECT_NEAR(meanOfAllPoints.z(), mean, 3 * stdDev);
  }

  if (::testing::Test::HasFailure()) {
    std::cout << "\n Test MeanPositionTest failed with seed: " << seed
              << std::endl;
  }
}

}  // namespace grid_map_pcl_test
}  // namespace grid_map

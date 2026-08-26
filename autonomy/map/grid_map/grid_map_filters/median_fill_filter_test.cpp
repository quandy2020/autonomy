#include <cmath>
#include <limits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_core/utils/testing.hpp"
#include "autonomy/map/grid_map/grid_map_filters/median_fill_filter.hpp"

using namespace grid_map;
using namespace ::testing;

TEST(MedianFillFilter, ConstructFilterTest) {
  MedianFillFilter medianFillFilter{};
  SUCCEED();
}

TEST(MedianFillFilter, LoadParametersAndUpdateTest) {
  MedianFillFilter medianFillFilter{};

  YAML::Node params = YAML::Load(R"(
input_layer: elevation
output_layer: elevation_filtered
fill_hole_radius: 0.02
filter_existing_values: true
existing_value_radius: 0.02
fill_mask_layer: fill_mask
debug: false
num_erode_dilation_iterations: 4
)");
  ASSERT_TRUE(medianFillFilter.configure(params, "median",
                                         "grid_map/MedianFillFilter"));

  GridMap filterInput = GridMap({"elevation", "variance"});
  filterInput.setGeometry(Length(0.4, 0.4), 0.02, Position(1.0, 5.0));
  filterInput.setFrameId("map");

  Matrix& elevationLayer = filterInput["elevation"];
  Matrix& varianceLayer = filterInput["variance"];
  elevationLayer.setConstant(1);
  varianceLayer.setConstant(0.05);

  GridMap filterOutput;
  ASSERT_TRUE(medianFillFilter.update(filterInput, filterOutput));

  ASSERT_THAT(filterOutput.getLayers(),
              ElementsAre(StrEq("elevation"), StrEq("variance"),
                          StrEq("elevation_filtered"), StrEq("fill_mask")));

  ASSERT_TRUE(filterInput["elevation"].isApprox(filterOutput["elevation"]));
  ASSERT_TRUE(filterInput["variance"].isApprox(filterOutput["variance"]));
  ASSERT_TRUE(filterOutput["fill_mask"].isApprox(
      Matrix::Ones(filterOutput.getSize().x(), filterOutput.getSize().y())));

  filterInput["elevation"].bottomRightCorner<5, 5>().setConstant(NAN);
  GridMap noisyFilterInput{filterInput};
  noisyFilterInput["elevation"](0, 0) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](10, 5) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](2, 4) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](6, 3) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](11, 8) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](3, 15) = std::numeric_limits<float>::quiet_NaN();
  noisyFilterInput["elevation"](10, 10) = std::numeric_limits<float>::quiet_NaN();

  GridMap noisyFilterOutput;
  ASSERT_TRUE(medianFillFilter.update(noisyFilterInput, noisyFilterOutput));
  ASSERT_MATRICES_EQ_WITH_NAN(filterInput["elevation"],
                              noisyFilterOutput["elevation_filtered"]);
}

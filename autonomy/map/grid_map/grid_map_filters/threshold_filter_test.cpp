#include <cmath>
#include <limits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_core/utils/testing.hpp"
#include "autonomy/map/grid_map/grid_map_filters/threshold_filter.hpp"

using namespace grid_map;
using namespace ::testing;

TEST(ThresholdFilter, LoadParametersAndUpdateTest) {
  ThresholdFilter thresholdFilter{};

  YAML::Node params = YAML::Load(R"(
condition_layer: standard_deviation
output_layer: standard_deviation_filtered
upper_threshold: 0.05
set_to: .nan
)");
  ASSERT_TRUE(thresholdFilter.configure(params, "threshold_filter",
                                        "grid_map/ThresholdFilter"));

  GridMap filterInput = GridMap({"standard_deviation", "standard_deviation_filtered"});
  filterInput.setGeometry(Length(0.4, 0.4), 0.02, Position(1.0, 5.0));
  filterInput.setFrameId("map");

  Matrix& conditionLayer = filterInput["standard_deviation"];
  conditionLayer.setConstant(0.03);
  conditionLayer.bottomRightCorner<5, 5>().setConstant(0.06);
  conditionLayer.topLeftCorner<3, 3>().setConstant(NAN);

  Matrix& outputLayer = filterInput["standard_deviation_filtered"];
  outputLayer.setConstant(1.0);

  Matrix outputLayerExpected{outputLayer.rows(), outputLayer.cols()};
  outputLayerExpected.setConstant(1.0f);
  outputLayerExpected.topLeftCorner<3, 3>().setConstant(NAN);
  outputLayerExpected.bottomRightCorner<5, 5>().setConstant(NAN);

  GridMap filterOutput;
  ASSERT_TRUE(thresholdFilter.update(filterInput, filterOutput));

  ASSERT_THAT(filterOutput.getLayers(),
              ElementsAre(StrEq("standard_deviation"), StrEq("standard_deviation_filtered")));
  ASSERT_MATRICES_EQ_WITH_NAN(conditionLayer, filterOutput["standard_deviation"]);
  ASSERT_MATRICES_EQ_WITH_NAN(filterOutput["standard_deviation_filtered"], outputLayerExpected);
}

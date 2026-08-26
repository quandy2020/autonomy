/*
 * grid_map_converter_test.cpp
 */

#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/grid_map/grid_map_costmap_2d/costmap_2d_converter.hpp"

namespace grid_map {
namespace {

TEST(GridMapConverter, RoundTripProto) {
  GridMap map({"elevation"});
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.setFrameId("map");
  map.setTimestamp(123456789ULL);
  map["elevation"].setConstant(1.5f);
  map.at("elevation", Index(0, 0)) = 2.5f;

  automsgs::msgs::map_msgs::GridMap message;
  GridMapConverter::toMessage(map, message);
  ASSERT_EQ(message.layers_size(), 1);
  EXPECT_EQ(message.layers(0), "elevation");

  GridMap restored;
  ASSERT_TRUE(GridMapConverter::fromMessage(message, restored));
  EXPECT_EQ(restored.getFrameId(), "map");
  EXPECT_EQ(restored.getTimestamp(), 123456789ULL);
  EXPECT_FLOAT_EQ(restored.at("elevation", Index(0, 0)), 2.5f);
  EXPECT_FLOAT_EQ(restored.at("elevation", Index(1, 1)), 1.5f);
}

TEST(GridMapConverter, OccupancyGridRoundTrip) {
  GridMap map({"occupancy"});
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(1.0, 1.0));
  map.setFrameId("map");
  map["occupancy"](0, 0) = 0.0f;
  map["occupancy"](0, 1) = 50.0f;
  map["occupancy"](1, 0) = 100.0f;
  map["occupancy"](1, 1) = std::numeric_limits<float>::quiet_NaN();

  automsgs::msgs::map_msgs::OccupancyGrid occupancy;
  GridMapConverter::toOccupancyGrid(map, "occupancy", 0.0f, 100.0f, occupancy);
  ASSERT_EQ(occupancy.info().width(), 2u);
  ASSERT_EQ(occupancy.info().height(), 2u);
  ASSERT_EQ(occupancy.data_size(), 4);

  GridMap restored;
  ASSERT_TRUE(
      GridMapConverter::fromOccupancyGrid(occupancy, "occupancy", restored));
  EXPECT_NEAR(restored.at("occupancy", Index(0, 0)), 0.0f, 1e-3);
  EXPECT_NEAR(restored.at("occupancy", Index(0, 1)), 50.0f, 1e-3);
  EXPECT_NEAR(restored.at("occupancy", Index(1, 0)), 100.0f, 1e-3);
  EXPECT_TRUE(std::isnan(restored.at("occupancy", Index(1, 1))));
}

TEST(Costmap2DConverter, Bidirectional) {
  GridMap map({"cost"});
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(1.0, 1.0));
  map["cost"].setConstant(0.0f);
  map.at("cost", Index(0, 0)) = 100.0f;

  Costmap2DConverter<GridMap> converter;
  autonomy::map::costmap_2d::Costmap2D costmap;
  converter.initializeFromGridMap(map, costmap);
  ASSERT_TRUE(converter.setCostmap2DFromGridMap(map, "cost", costmap));

  GridMap restored({"cost"});
  converter.initializeFromCostmap2D(costmap, restored);
  ASSERT_TRUE(converter.addLayerFromCostmap2D(costmap, "cost", restored));
  EXPECT_NEAR(restored.at("cost", Index(0, 0)), 100.0f, 1e-3);
}

TEST(GridMapConverter, ToPointCloud) {
  GridMap map({"elevation"});
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.setFrameId("map");
  map["elevation"].setConstant(1.0f);
  map.at("elevation", Index(0, 0)) = 2.0f;

  automsgs::msgs::sensor_msgs::PointCloud2 cloud;
  GridMapConverter::toPointCloud(map, "elevation", cloud);
  EXPECT_EQ(cloud.header().frame_id(), "map");
  EXPECT_EQ(cloud.height(), 1u);
  EXPECT_EQ(cloud.width(), 4u);
  ASSERT_EQ(cloud.fields_size(), 3);
  EXPECT_EQ(cloud.fields(0).name(), "x");
  EXPECT_EQ(cloud.fields(1).name(), "y");
  EXPECT_EQ(cloud.fields(2).name(), "z");
}

TEST(GridMapConverter, ImageRoundTripMono8) {
  GridMap map({"elevation"});
  map.setGeometry(Length(2.0, 2.0), 1.0, Position(0.0, 0.0));
  map.setFrameId("map");
  map["elevation"].setConstant(0.0f);
  map.at("elevation", Index(0, 0)) = 1.0f;
  map.at("elevation", Index(1, 1)) = 0.5f;

  automsgs::msgs::sensor_msgs::Image image;
  ASSERT_TRUE(GridMapConverter::toImage(map, "elevation", "mono8", 0.0f, 1.0f,
                                        image));
  EXPECT_EQ(image.encoding(), "mono8");
  EXPECT_EQ(image.width(), 2u);
  EXPECT_EQ(image.height(), 2u);

  GridMap restored;
  ASSERT_TRUE(GridMapConverter::initializeFromImage(image, 1.0, restored));
  ASSERT_TRUE(GridMapConverter::addLayerFromImage(image, "elevation", restored,
                                                  0.0f, 1.0f));
  EXPECT_NEAR(restored.at("elevation", Index(0, 0)), 1.0f, 1.0 / 255.0 + 1e-3);
  EXPECT_NEAR(restored.at("elevation", Index(1, 1)), 0.5f, 1.0 / 255.0 + 1e-3);
}

}  // namespace
}  // namespace grid_map

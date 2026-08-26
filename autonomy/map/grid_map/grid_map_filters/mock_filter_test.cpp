#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_filters/mock_filter.hpp"

using namespace grid_map;

TEST(MockFilter, ConstructAndUpdate) {
  MockFilter filter;
  YAML::Node params = YAML::Load("{processing_time: 1, print_name: false}");
  ASSERT_TRUE(filter.configure(params, "mock", "grid_map/MockFilter"));
  GridMap in({"elevation"});
  in.setGeometry(Length(1.0, 1.0), 0.1);
  GridMap out;
  ASSERT_TRUE(filter.update(in, out));
  EXPECT_EQ(out.getLayers().size(), 1u);
}

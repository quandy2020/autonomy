#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_filters/filter_factory.hpp"
#include "autonomy/map/grid_map/grid_map_filters/filters/filter_chain.hpp"

using namespace grid_map;

TEST(FilterFactory, CreateKnownTypes) {
  EXPECT_NE(FilterFactory::create("ThresholdFilter"), nullptr);
  EXPECT_NE(FilterFactory::create("grid_map/MedianFillFilter"), nullptr);
  EXPECT_NE(FilterFactory::create("InpaintFilter"), nullptr);
  EXPECT_EQ(FilterFactory::create("DoesNotExist"), nullptr);
}

TEST(FilterFactory, ChainFromYaml) {
  YAML::Node chain = YAML::Load(R"(
- name: dup
  type: DuplicationFilter
  params:
    input_layer: elevation
    output_layer: elevation_copy
)");
  filters::FilterChain<GridMap> filterChain;
  ASSERT_TRUE(filterChain.configure(chain, [](const std::string& type) {
    return FilterFactory::create(type);
  }));
  EXPECT_EQ(filterChain.size(), 1u);

  GridMap in({"elevation"});
  in.setGeometry(Length(1.0, 1.0), 0.5);
  in["elevation"].setConstant(2.0f);
  GridMap out;
  ASSERT_TRUE(filterChain.update(in, out));
  ASSERT_TRUE(out.exists("elevation_copy"));
  EXPECT_FLOAT_EQ(out.at("elevation_copy", Index(0, 0)), 2.0f);
}

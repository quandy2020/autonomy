/*
 * filter_factory.cpp
 */

#include "autonomy/map/grid_map/grid_map_filters/filter_factory.hpp"

#include <cstring>

#include "autonomy/map/grid_map/grid_map_cv/inpaint_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/buffer_normalizer_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/color_blending_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/color_fill_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/color_map_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/curvature_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/deletion_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/duplication_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/light_intensity_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/math_expression_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/mean_in_radius_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/median_fill_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/min_in_radius_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/mock_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/normal_color_map_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/normal_vectors_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/set_basic_layers_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/sliding_window_math_expression_filter.hpp"
#include "autonomy/map/grid_map/grid_map_filters/threshold_filter.hpp"

namespace grid_map {
namespace {

std::string normalizeType(const std::string& type) {
  // Accept ROS plugin-style prefixes from original grid_map demos.
  static const char* kPrefixes[] = {
      "grid_map/", "gridMapFilters/", "gridMapCv/", "grid_map_filters/",
      "grid_map_cv/"};
  for (const char* prefix : kPrefixes) {
    const size_t len = std::strlen(prefix);
    if (type.rfind(prefix, 0) == 0) {
      return type.substr(len);
    }
  }
  return type;
}

template <typename FilterT>
std::unique_ptr<filters::FilterBase<GridMap>> make() {
  return std::make_unique<FilterT>();
}

}  // namespace

std::unique_ptr<filters::FilterBase<GridMap>> FilterFactory::create(
    const std::string& type) {
  const std::string name = normalizeType(type);

  if (name == "ThresholdFilter") {
    return make<ThresholdFilter>();
  }
  if (name == "MedianFillFilter") {
    return make<MedianFillFilter>();
  }
  if (name == "MeanInRadiusFilter") {
    return make<MeanInRadiusFilter>();
  }
  if (name == "MinInRadiusFilter") {
    return make<MinInRadiusFilter>();
  }
  if (name == "NormalVectorsFilter") {
    return make<NormalVectorsFilter>();
  }
  if (name == "NormalColorMapFilter") {
    return make<NormalColorMapFilter>();
  }
  if (name == "CurvatureFilter") {
    return make<CurvatureFilter>();
  }
  if (name == "MathExpressionFilter") {
    return make<MathExpressionFilter>();
  }
  if (name == "SlidingWindowMathExpressionFilter") {
    return make<SlidingWindowMathExpressionFilter>();
  }
  if (name == "DuplicationFilter") {
    return make<DuplicationFilter>();
  }
  if (name == "DeletionFilter") {
    return make<DeletionFilter>();
  }
  if (name == "ColorFillFilter") {
    return make<ColorFillFilter>();
  }
  if (name == "ColorMapFilter") {
    return make<ColorMapFilter>();
  }
  if (name == "ColorBlendingFilter") {
    return make<ColorBlendingFilter>();
  }
  if (name == "LightIntensityFilter") {
    return make<LightIntensityFilter>();
  }
  if (name == "BufferNormalizerFilter") {
    return make<BufferNormalizerFilter>();
  }
  if (name == "SetBasicLayersFilter") {
    return make<SetBasicLayersFilter>();
  }
  if (name == "MockFilter") {
    return make<MockFilter>();
  }
  if (name == "InpaintFilter") {
    return make<InpaintFilter>();
  }
  return nullptr;
}

}  // namespace grid_map

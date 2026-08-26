/*
 * InpaintFilter.cpp
 *
 * Adapted for autonomy (ROS-free).
 */

#include "autonomy/map/grid_map/grid_map_cv/inpaint_filter.hpp"

#include <opencv2/opencv.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map_core.hpp"
#include "autonomy/map/grid_map/grid_map_cv/grid_map_cv_converter.hpp"

using namespace filters;

namespace grid_map {

InpaintFilter::InpaintFilter() : radius_(5.0) {}

InpaintFilter::~InpaintFilter() = default;

bool InpaintFilter::configure() {
  if (!FilterBase::getParam(std::string("radius"), radius_)) {
    AERROR << "InpaintRadius filter did not find param radius.";
    return false;
  }
  if (radius_ < 0.0) {
    AERROR << "Radius must be greater than zero.";
    return false;
  }
  ADEBUG << "Radius = " << radius_;

  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    AERROR << "Inpaint filter did not find parameter `input_layer`.";
    return false;
  }
  ADEBUG << "Inpaint input layer is = " << inputLayer_;

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    AERROR << "Inpaint filter did not find parameter `output_layer`.";
    return false;
  }
  ADEBUG << "Inpaint output layer = " << outputLayer_;
  return true;
}

bool InpaintFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  mapOut.add(outputLayer_);
  mapOut.add("inpaint_mask", 0.0);
  mapOut.setBasicLayers(std::vector<std::string>());

  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (!mapOut.isValid(*iterator, inputLayer_)) {
      mapOut.at("inpaint_mask", *iterator) = 1.0;
    }
  }

  cv::Mat originalImage;
  cv::Mat mask;
  cv::Mat filledImage;
  const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
  const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();

  GridMapCvConverter::toImage<unsigned char, 3>(
      mapOut, inputLayer_, CV_8UC3, minValue, maxValue, originalImage);
  GridMapCvConverter::toImage<unsigned char, 1>(mapOut, "inpaint_mask",
                                                CV_8UC1, mask);

  const double radiusInPixels = radius_ / mapIn.getResolution();
  cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);

  GridMapCvConverter::addLayerFromImage<unsigned char, 3>(
      filledImage, outputLayer_, mapOut, minValue, maxValue);
  mapOut.erase("inpaint_mask");
  return true;
}

}  // namespace grid_map

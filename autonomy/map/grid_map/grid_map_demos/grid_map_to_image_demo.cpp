/*
 * gridmap_to_image_demo.cpp
 * Usage: gridmap_to_image_demo [map.pb|image.png] [out.png]
 * If input is png, convert via image_to_gridmap path first (elevation layer).
 */

#include <opencv2/imgcodecs.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_cv/grid_map_cv_converter.hpp"
#include "autonomy/map/grid_map/grid_map_demos/helpers.hpp"
#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"

using namespace grid_map;

int main(int argc, char** argv) {
  const auto in_path = std::filesystem::path(grid_map::grid_map_demos::requireArg(
      argc, argv, 1,
      grid_map::grid_map_demos::resolveDemoPath("data/terrain.png").string().c_str()));
  const auto out_path = std::filesystem::path(grid_map::grid_map_demos::requireArg(
      argc, argv, 2, "/tmp/grid_map_demos/gridmap_to_image/elevation.png"));

  GridMap map;
  if (in_path.extension() == ".pb") {
    if (!GridMapConverter::loadFromFile(in_path.string(), map)) {
      AERROR << "Failed to load " << in_path;
      return 1;
    }
  } else {
    cv::Mat image = cv::imread(in_path.string(), cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
      AERROR << "Failed to read " << in_path;
      return 1;
    }
    GridMapCvConverter::initializeFromImage(image, 0.03, map, Position::Zero());
    GridMapCvConverter::addLayerFromImage<unsigned char, 1>(image, "elevation",
                                                            map, 0.0f, 1.0f);
  }

  if (!map.exists("elevation")) {
    AERROR << "No elevation layer";
    return 1;
  }
  cv::Mat out;
  if (!GridMapCvConverter::toImage<unsigned char, 1>(map, "elevation", CV_8UC1,
                                                    out)) {
    return 1;
  }
  std::filesystem::create_directories(out_path.parent_path());
  if (!cv::imwrite(out_path.string(), out)) {
    AERROR << "imwrite failed: " << out_path;
    return 1;
  }
  AINFO << "Wrote " << out_path;
  return 0;
}

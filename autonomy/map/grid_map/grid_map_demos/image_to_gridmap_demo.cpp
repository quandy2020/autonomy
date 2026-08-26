/*
 * image_to_gridmap_demo.cpp
 * Usage: image_to_gridmap_demo [image.png] [out_dir] [resolution=0.03]
 */

#include <opencv2/imgcodecs.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_cv/grid_map_cv_converter.hpp"
#include "autonomy/map/grid_map/grid_map_demos/helpers.hpp"

using namespace grid_map;

int main(int argc, char** argv) {
  const auto image_path = std::filesystem::path(grid_map::grid_map_demos::requireArg(
      argc, argv, 1,
      grid_map::grid_map_demos::resolveDemoPath("data/terrain.png").string().c_str()));
  const auto out_dir = std::filesystem::path(grid_map::grid_map_demos::requireArg(
      argc, argv, 2, "/tmp/grid_map_demos/image_to_gridmap"));
  const double resolution = std::atof(
      grid_map::grid_map_demos::requireArg(argc, argv, 3, "0.03").c_str());

  cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
  if (image.empty()) {
    AERROR << "Failed to read image: " << image_path;
    return 1;
  }

  GridMap map;
  if (!GridMapCvConverter::initializeFromImage(image, resolution, map,
                                               Position::Zero())) {
    AERROR << "initializeFromImage failed";
    return 1;
  }
  map.setFrameId("map");
  map.setBasicLayers({"elevation"});

  if (image.channels() >= 3) {
    GridMapCvConverter::addLayerFromImage<unsigned char, 3>(
        image, "elevation", map, 0.0f, 1.0f);
    GridMapCvConverter::addColorLayerFromImage<unsigned char, 3>(image, "color",
                                                                 map);
  } else {
    GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
        image, "elevation", map, 0.0f, 1.0f);
  }

  AINFO << "Map size " << map.getLength().transpose() << " m, cells "
        << map.getSize().transpose();
  grid_map::grid_map_demos::saveLayerAsPng(map, "elevation", out_dir / "elevation.png");
  grid_map::grid_map_demos::saveMapProto(map, out_dir / "map.pb");
  return 0;
}

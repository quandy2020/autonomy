/*
 * GridMapLoader.cpp
 */

#include "autonomy/map/grid_map/grid_map_loader/grid_map_loader.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"

namespace grid_map_loader {

bool GridMapLoader::load() {
  if (filePath_.empty()) {
    AERROR << "GridMapLoader: file path is empty.";
    return false;
  }
  AINFO << "Loading grid map from path " << filePath_;
  if (!grid_map::GridMapConverter::loadFromFile(filePath_, map_)) {
    AERROR << "GridMapLoader: failed to load " << filePath_;
    return false;
  }
  return true;
}

}  // namespace grid_map_loader

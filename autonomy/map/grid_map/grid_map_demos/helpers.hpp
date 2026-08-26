/*
 * helpers.hpp
 *
 * Shared helpers for ROS-free grid_map demos.
 */

#pragma once

#include <cstdlib>
#include <filesystem>
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_cv/grid_map_cv_converter.hpp"
#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"

namespace grid_map {
namespace grid_map_demos {

inline std::filesystem::path demosRoot() {
  return std::filesystem::path(__FILE__).parent_path();
}

inline std::filesystem::path resolveDemoPath(const std::string& relative) {
  const auto from_source = demosRoot() / relative;
  if (std::filesystem::exists(from_source)) {
    return from_source;
  }
  if (const char* prefix = std::getenv("AUTONOMY_PREFIX")) {
    const auto installed =
        std::filesystem::path(prefix) /
        "share/autonomy/map/grid_map/grid_map_demos" / relative;
    if (std::filesystem::exists(installed)) {
      return installed;
    }
  }
  return from_source;
}

inline std::string requireArg(int argc, char** argv, int index,
                              const char* fallback) {
  if (argc > index && argv[index] != nullptr && argv[index][0] != '\0') {
    return argv[index];
  }
  return fallback;
}

inline bool saveLayerAsPng(const grid_map::GridMap& map,
                           const std::string& layer,
                           const std::filesystem::path& path) {
  if (!map.exists(layer)) {
    AERROR << "Layer missing: " << layer;
    return false;
  }
  cv::Mat image;
  if (!grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
          map, layer, CV_8UC1, image)) {
    AERROR << "Failed to convert layer to image: " << layer;
    return false;
  }
  std::filesystem::create_directories(path.parent_path());
  if (!cv::imwrite(path.string(), image)) {
    AERROR << "Failed to write: " << path;
    return false;
  }
  AINFO << "Wrote " << path;
  return true;
}

inline bool saveMapProto(const grid_map::GridMap& map,
                         const std::filesystem::path& path) {
  std::filesystem::create_directories(path.parent_path());
  if (!grid_map::GridMapConverter::saveToFile(map, path.string())) {
    AERROR << "Failed to save protobuf map: " << path;
    return false;
  }
  AINFO << "Wrote " << path;
  return true;
}

}  // namespace grid_map_demos
}  // namespace grid_map

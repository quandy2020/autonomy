/*
 * sdf_demo.cpp — build SignedDistanceField and export PointCloud2 counts.
 * Usage: sdf_demo [image.png] [out_dir]
 */

#include <cmath>
#include <fstream>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <opencv2/imgcodecs.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_cv/grid_map_cv_converter.hpp"
#include "autonomy/map/grid_map/grid_map_demos/helpers.hpp"
#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"
#include "autonomy/map/grid_map/grid_map_sdf/signed_distance_field.hpp"

using namespace grid_map;

int main(int argc, char** argv) {
  const auto image_path = std::filesystem::path(grid_map::grid_map_demos::requireArg(
      argc, argv, 1,
      grid_map::grid_map_demos::resolveDemoPath("data/terrain.png").string().c_str()));
  const auto out_dir = std::filesystem::path(
      grid_map::grid_map_demos::requireArg(argc, argv, 2, "/tmp/grid_map_demos/sdf"));

  cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
  if (image.empty()) {
    AERROR << "Failed to read " << image_path;
    return 1;
  }

  GridMap map;
  GridMapCvConverter::initializeFromImage(image, 0.05, map, Position::Zero());
  map.setFrameId("map");
  GridMapCvConverter::addLayerFromImage<unsigned char, 1>(image, "elevation",
                                                          map, -0.5f, 0.5f);
  auto& elevation = map.get("elevation");
  if (elevation.hasNaN()) {
    const float fill = elevation.minCoeffOfFinites();
    elevation = elevation.unaryExpr(
        [=](float v) { return std::isfinite(v) ? v : fill; });
  }

  const float minValue = elevation.minCoeffOfFinites() - 0.1f;
  const float maxValue = elevation.maxCoeffOfFinites() + 0.1f;
  SignedDistanceField sdf(map, "elevation", minValue, maxValue);

  automsgs::msgs::sensor_msgs::PointCloud2 full, free_space, occupied;
  GridMapConverter::toPointCloud(sdf, full);
  GridMapConverter::toPointCloud(sdf, free_space, 1,
                                 [](float v) { return v > 0.0f; });
  GridMapConverter::toPointCloud(sdf, occupied, 1,
                                 [](float v) { return v <= 0.0f; });

  AINFO << "SDF voxels=" << sdf.size() << " full_cloud=" << full.width()
        << " free=" << free_space.width() << " occupied=" << occupied.width();

  std::filesystem::create_directories(out_dir);
  {
    std::ofstream out(out_dir / "full_sdf.pc2.pb", std::ios::binary);
    full.SerializeToOstream(&out);
  }
  {
    std::ofstream out(out_dir / "free_space.pc2.pb", std::ios::binary);
    free_space.SerializeToOstream(&out);
  }
  {
    std::ofstream out(out_dir / "occupied.pc2.pb", std::ios::binary);
    occupied.SerializeToOstream(&out);
  }
  grid_map::grid_map_demos::saveMapProto(map, out_dir / "elevation_map.pb");
  return 0;
}

/*
 * resolution_change_demo.cpp
 */

#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_cv/grid_map_cv_processing.hpp"
#include "autonomy/map/grid_map/grid_map_demos/helpers.hpp"

using namespace grid_map;

int main(int argc, char** argv) {
  const int frames =
      std::atoi(grid_map::grid_map_demos::requireArg(argc, argv, 1, "10").c_str());
  const auto out_dir = std::filesystem::path(grid_map::grid_map_demos::requireArg(
      argc, argv, 2, "/tmp/grid_map_demos/resolution"));

  GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03);
  map["elevation"].setZero();

  Polygon polygon;
  polygon.setFrameId(map.getFrameId());
  polygon.addVertex(Position(0.480, 0.000));
  polygon.addVertex(Position(0.164, 0.155));
  polygon.addVertex(Position(0.116, 0.500));
  polygon.addVertex(Position(-0.133, 0.250));
  polygon.addVertex(Position(-0.480, 0.399));
  polygon.addVertex(Position(-0.316, 0.000));
  polygon.addVertex(Position(-0.480, -0.399));
  polygon.addVertex(Position(-0.133, -0.250));
  polygon.addVertex(Position(0.116, -0.500));
  polygon.addVertex(Position(0.164, -0.155));
  polygon.addVertex(Position(0.480, 0.000));
  for (PolygonIterator iterator(map, polygon); !iterator.isPastEnd();
       ++iterator) {
    map.at("elevation", *iterator) = 0.3f;
  }

  for (int frame = 0; frame < frames; ++frame) {
    const double t = frame / 10.0;
    const double resolution = 0.05 + 0.04 * std::sin(t);
    GridMap modifiedMap;
    if (!GridMapCvProcessing::changeResolution(map, modifiedMap, resolution)) {
      AERROR << "changeResolution failed at " << resolution;
      return 1;
    }
    AINFO << "Resolution " << resolution << " -> size "
          << modifiedMap.getSize().transpose();
    if (frame == frames - 1) {
      grid_map::grid_map_demos::saveLayerAsPng(modifiedMap, "elevation",
                                     out_dir / "elevation.png");
      grid_map::grid_map_demos::saveMapProto(modifiedMap, out_dir / "map.pb");
    }
  }
  return 0;
}

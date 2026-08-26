/*
 * simple_demo.cpp — ROS-free port of grid_map_demos/simple_demo_node.
 * Runs a fixed number of elevation updates and writes outputs.
 */

#include <cmath>
#include <cstdint>
#include <filesystem>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_demos/helpers.hpp"

using namespace grid_map;

int main(int argc, char** argv) {
  const int frames =
      std::atoi(grid_map::grid_map_demos::requireArg(argc, argv, 1, "30").c_str());
  const auto out_dir = std::filesystem::path(
      grid_map::grid_map_demos::requireArg(argc, argv, 2, "/tmp/grid_map_demos/simple"));

  GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03);
  AINFO << "Created map with size " << map.getLength().x() << " x "
        << map.getLength().y() << " m (" << map.getSize()(0) << " x "
        << map.getSize()(1) << " cells).";

  for (int frame = 0; frame < frames; ++frame) {
    const double t = frame / 30.0;
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) =
          -0.04 + 0.2 * std::sin(3.0 * t + 5.0 * position.y()) * position.x();
    }
    map.setTimestamp(static_cast<uint64_t>(t * 1e9));
  }

  grid_map::grid_map_demos::saveLayerAsPng(map, "elevation", out_dir / "elevation.png");
  grid_map::grid_map_demos::saveMapProto(map, out_dir / "map.pb");
  return 0;
}

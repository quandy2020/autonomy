/*
 * move_demo.cpp — compare GridMap::move vs setPosition offline.
 */

#include <cmath>
#include <cstdint>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_demos/helpers.hpp"

using namespace grid_map;

int main(int argc, char** argv) {
  const int steps =
      std::atoi(grid_map::grid_map_demos::requireArg(argc, argv, 1, "100").c_str());
  const auto out_dir = std::filesystem::path(
      grid_map::grid_map_demos::requireArg(argc, argv, 2, "/tmp/grid_map_demos/move"));

  GridMap map({"layer"});
  map.setFrameId("map");
  map.setGeometry(Length(0.7, 0.7), 0.01, Position(0.0, 0.0));
  map["layer"].setRandom();

  auto run = [&](bool useMove, const char* name) {
    GridMap tempMap(map);
    for (int i = 0; i < steps; ++i) {
      const double t = i / 10.0;
      const Position newPosition = 0.03 * t * Position(std::cos(t), std::sin(t));
      if (useMove) {
        tempMap.move(newPosition);
      } else {
        tempMap.setPosition(newPosition);
      }
      tempMap.setTimestamp(static_cast<uint64_t>(t * 1e9));
    }
    AINFO << name << " final position "
          << tempMap.getPosition().transpose();
    grid_map::grid_map_demos::saveLayerAsPng(tempMap, "layer",
                                   out_dir / (std::string(name) + ".png"));
    grid_map::grid_map_demos::saveMapProto(tempMap, out_dir / (std::string(name) + ".pb"));
  };

  run(true, "move");
  run(false, "set_position");
  return 0;
}

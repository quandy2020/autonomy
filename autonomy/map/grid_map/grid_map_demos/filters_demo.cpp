/*
 * filters_demo.cpp
 * Offline filter-chain demo: synthetic elevation (+ holes) → YAML chain → outputs.
 *
 * Usage: filters_demo [filter_chain.yaml] [out_dir]
 */

#include <cmath>
#include <limits>

#include <yaml-cpp/yaml.h>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_demos/helpers.hpp"
#include "autonomy/map/grid_map/grid_map_filters/filter_factory.hpp"
#include "autonomy/map/grid_map/grid_map_filters/filters/filter_chain.hpp"

using namespace grid_map;

int main(int argc, char** argv) {
  const auto chain_path = std::filesystem::path(grid_map::grid_map_demos::requireArg(
      argc, argv, 1,
      grid_map::grid_map_demos::resolveDemoPath("config/filters_demo_filter_chain.yaml")
          .string()
          .c_str()));
  const auto out_dir = std::filesystem::path(
      grid_map::grid_map_demos::requireArg(argc, argv, 2, "/tmp/grid_map_demos/filters"));

  GridMap map({"elevation", "color"});
  map.setFrameId("map");
  map.setGeometry(Length(2.0, 2.0), 0.02, Position(0.0, 0.0));
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Position p;
    map.getPosition(*it, p);
    map.at("elevation", *it) =
        0.1f * static_cast<float>(std::sin(4.0 * p.x()) * std::cos(3.0 * p.y()));
    map.at("color", *it) = 0.0f;
  }
  // Punch holes for inpaint.
  for (int i = 0; i < 80; ++i) {
    Position randomPosition = Position::Random();
    if (map.isInside(randomPosition)) {
      map.atPosition("elevation", randomPosition) =
          std::numeric_limits<float>::quiet_NaN();
    }
  }

  YAML::Node root = YAML::LoadFile(chain_path.string());
  if (!root) {
    AERROR << "Failed to load YAML: " << chain_path;
    return 1;
  }
  YAML::Node chain_node = root;
  if (root.IsMap() && root["grid_map_filters"]) {
    chain_node = root["grid_map_filters"];
  }
  if (!chain_node.IsSequence()) {
    AERROR << "Filter chain YAML must be a sequence (got type "
           << chain_node.Type() << ") in " << chain_path;
    return 1;
  }

  filters::FilterChain<GridMap> chain;
  if (!chain.configure(chain_node, [](const std::string& type) {
        auto filter = FilterFactory::create(type);
        if (!filter) {
          AERROR << "FilterFactory::create failed for type='" << type << "'";
        }
        return filter;
      })) {
    AERROR << "Failed to configure filter chain from " << chain_path;
    return 1;
  }

  GridMap output;
  if (!chain.update(map, output)) {
    AERROR << "Filter chain update failed.";
    return 1;
  }

  for (const auto& layer : output.getLayers()) {
    grid_map::grid_map_demos::saveLayerAsPng(output, layer,
                                   out_dir / (layer + ".png"));
  }
  grid_map::grid_map_demos::saveMapProto(output, out_dir / "filtered_map.pb");
  AINFO << "Filters demo wrote " << output.getLayers().size() << " layers to "
        << out_dir;
  return 0;
}

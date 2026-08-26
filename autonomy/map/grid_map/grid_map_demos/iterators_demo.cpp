/*
 * iterators_demo.cpp — ROS-free iterator demos (no RViz sleep/publish).
 */

#include <cmath>
#include <string>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_demos/helpers.hpp"

using namespace grid_map;

namespace {

void save(GridMap& map, const std::filesystem::path& out_dir,
          const std::string& name) {
  grid_map::grid_map_demos::saveLayerAsPng(map, "type", out_dir / (name + ".png"));
}

}  // namespace

int main(int argc, char** argv) {
  const auto out_dir = std::filesystem::path(
      grid_map::grid_map_demos::requireArg(argc, argv, 1, "/tmp/grid_map_demos/iterators"));

  GridMap map({"type"});
  map.setGeometry(Length(1.0, 1.0), 0.05, Position(0.0, 0.0));
  map.setFrameId("map");

  // GridMapIterator
  map.clearAll();
  {
    Matrix& data = map["type"];
    for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      data(iterator.getLinearIndex()) = 1.0f;
    }
  }
  save(map, out_dir, "grid_map_iterator");

  // SubmapIterator
  map.clearAll();
  for (SubmapIterator iterator(map, Index(3, 5), Index(12, 7));
       !iterator.isPastEnd(); ++iterator) {
    map.at("type", *iterator) = 1.0f;
  }
  save(map, out_dir, "submap_iterator");

  // CircleIterator
  map.clearAll();
  for (CircleIterator iterator(map, Position(0.0, -0.15), 0.4);
       !iterator.isPastEnd(); ++iterator) {
    map.at("type", *iterator) = 1.0f;
  }
  save(map, out_dir, "circle_iterator");

  // EllipseIterator
  map.clearAll();
  for (EllipseIterator iterator(map, Position(0.0, -0.15), Length(0.45, 0.9),
                                M_PI_4);
       !iterator.isPastEnd(); ++iterator) {
    map.at("type", *iterator) = 1.0f;
  }
  save(map, out_dir, "ellipse_iterator");

  // SpiralIterator
  map.clearAll();
  for (SpiralIterator iterator(map, Position(0.0, -0.15), 0.4);
       !iterator.isPastEnd(); ++iterator) {
    map.at("type", *iterator) = 1.0f;
  }
  save(map, out_dir, "spiral_iterator");

  // LineIterator
  map.clearAll();
  for (LineIterator iterator(map, Index(18, 2), Index(2, 13));
       !iterator.isPastEnd(); ++iterator) {
    map.at("type", *iterator) = 1.0f;
  }
  save(map, out_dir, "line_iterator");

  // PolygonIterator
  map.clearAll();
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
    map.at("type", *iterator) = 1.0f;
  }
  save(map, out_dir, "polygon_iterator");

  // SlidingWindowIterator blur
  map.add("copy", map["type"]);
  for (SlidingWindowIterator iterator(
           map, "copy", SlidingWindowIterator::EdgeHandling::CROP, 3);
       !iterator.isPastEnd(); ++iterator) {
    map.at("type", *iterator) = iterator.getData().meanOfFinites();
  }
  save(map, out_dir, "sliding_window_iterator");

  AINFO << "Iterator demos written to " << out_dir;
  return 0;
}

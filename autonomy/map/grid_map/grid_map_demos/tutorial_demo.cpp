/*
 * tutorial_demo.cpp — ROS-free port of tutorial_demo_node.
 */

#include <cmath>
#include <cstdint>
#include <limits>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_demos/helpers.hpp"

using namespace grid_map;

int main(int argc, char** argv) {
  const auto out_dir = std::filesystem::path(
      grid_map::grid_map_demos::requireArg(argc, argv, 1, "/tmp/grid_map_demos/tutorial"));

  GridMap map({"elevation", "normal_x", "normal_y", "normal_z"});
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03, Position(0.0, -0.1));
  AINFO << "Created map " << map.getSize().transpose() << " cells, center "
        << map.getPosition().transpose();

  const double t = 1.0;
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    Position position;
    map.getPosition(*it, position);
    map.at("elevation", *it) =
        -0.04 + 0.2 * std::sin(3.0 * t + 5.0 * position.y()) * position.x();
    Eigen::Vector3d normal(
        -0.2 * std::sin(3.0 * t + 5.0 * position.y()),
        -position.x() * std::cos(3.0 * t + 5.0 * position.y()), 1.0);
    normal.normalize();
    map.at("normal_x", *it) = normal.x();
    map.at("normal_y", *it) = normal.y();
    map.at("normal_z", *it) = normal.z();
  }

  map.add("noise", 0.015 * Matrix::Random(map.getSize()(0), map.getSize()(1)));
  map.add("elevation_noisy", map.get("elevation") + map["noise"]);

  for (unsigned int i = 0; i < 500; ++i) {
    Position randomPosition = Position::Random();
    if (map.isInside(randomPosition)) {
      map.atPosition("elevation_noisy", randomPosition) =
          std::numeric_limits<float>::infinity();
    }
  }

  map.add("elevation_filtered", map.get("elevation_noisy"));
  Position topLeftCorner(1.0, 0.4);
  boundPositionToRange(topLeftCorner, map.getLength(), map.getPosition());
  Index startIndex;
  map.getIndex(topLeftCorner, startIndex);
  Size size = (Length(1.2, 0.8) / map.getResolution()).cast<int>();
  for (SubmapIterator it(map, startIndex, size); !it.isPastEnd(); ++it) {
    Position currentPosition;
    map.getPosition(*it, currentPosition);
    const double radius = 0.1;
    double mean = 0.0;
    double sumOfWeights = 0.0;
    for (CircleIterator circleIt(map, currentPosition, radius);
         !circleIt.isPastEnd(); ++circleIt) {
      if (!map.isValid(*circleIt, "elevation_noisy")) {
        continue;
      }
      Position currentPositionInCircle;
      map.getPosition(*circleIt, currentPositionInCircle);
      const double distance =
          (currentPosition - currentPositionInCircle).norm();
      const double weight = std::pow(radius - distance, 2);
      mean += weight * map.at("elevation_noisy", *circleIt);
      sumOfWeights += weight;
    }
    if (sumOfWeights > 0.0) {
      map.at("elevation_filtered", *it) = mean / sumOfWeights;
    }
  }

  map.add("error",
          (map.get("elevation_filtered") - map.get("elevation")).cwiseAbs());
  map.setTimestamp(static_cast<uint64_t>(t * 1e9));

  grid_map::grid_map_demos::saveLayerAsPng(map, "elevation", out_dir / "elevation.png");
  grid_map::grid_map_demos::saveLayerAsPng(map, "elevation_filtered",
                                 out_dir / "elevation_filtered.png");
  grid_map::grid_map_demos::saveLayerAsPng(map, "error", out_dir / "error.png");
  grid_map::grid_map_demos::saveMapProto(map, out_dir / "map.pb");
  return 0;
}

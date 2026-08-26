/*
 * normal_filter_comparison_demo.cpp — offline area vs raster normals.
 * Usage: normal_filter_comparison_demo [frames=20] [out_dir]
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

namespace {

void mapAddNoise(GridMap& map, const Size& gridMapSize, double noise_on_map) {
  map.add("noise",
          static_cast<float>(noise_on_map) *
              Matrix::Random(gridMapSize(0), gridMapSize(1)));
  map.add("elevation_noisy", map.get("elevation") + map["noise"]);
}

void mapAddOutliers(GridMap& map, const Size& gridMapSize,
                    double outlierPercentage) {
  const int numberInfPoints = static_cast<int>(
      outlierPercentage * gridMapSize(0) * gridMapSize(1));
  for (int i = 0; i < numberInfPoints; ++i) {
    Position randomPosition = Position::Random();
    if (map.isInside(randomPosition)) {
      map.atPosition("elevation_noisy", randomPosition) =
          std::numeric_limits<float>::infinity();
    }
  }
}

void mapAverageFiltering(GridMap& map, const Size& gridMapSize,
                         double filterRadius) {
  for (SubmapIterator it(map, Index(0, 0), gridMapSize); !it.isPastEnd(); ++it) {
    Position currentPosition;
    map.getPosition(*it, currentPosition);
    double mean = 0.0;
    double sumOfWeights = 0.0;
    for (CircleIterator circleIt(map, currentPosition, filterRadius);
         !circleIt.isPastEnd(); ++circleIt) {
      if (!map.isValid(*circleIt, "elevation_noisy")) {
        continue;
      }
      Position currentPositionInCircle;
      map.getPosition(*circleIt, currentPositionInCircle);
      const double distance =
          (currentPosition - currentPositionInCircle).norm();
      const double weight = std::pow(filterRadius - distance, 2);
      mean += weight * map.at("elevation_noisy", *circleIt);
      sumOfWeights += weight;
    }
    map.at("elevation_filtered", *it) =
        sumOfWeights != 0.0
            ? static_cast<float>(mean / sumOfWeights)
            : std::numeric_limits<float>::infinity();
  }
}

void normalsErrorCalculation(GridMap& map, const Size& gridMapSize,
                             double& directionalErrorAreaSum,
                             double& directionalErrorRasterSum) {
  map.add("directionalErrorArea", Matrix::Zero(gridMapSize(0), gridMapSize(1)));
  map.add("directionalErrorRaster",
          Matrix::Zero(gridMapSize(0), gridMapSize(1)));
  const Index submapStartIndex(1, 1);
  const Index submapBufferSize(gridMapSize(0) - 2, gridMapSize(1) - 2);
  for (SubmapIterator iterator(map, submapStartIndex, submapBufferSize);
       !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    Vector3 nA(map.at("normal_analytic_x", index),
               map.at("normal_analytic_y", index),
               map.at("normal_analytic_z", index));
    Vector3 nArea(map.at("normal_area_x", index), map.at("normal_area_y", index),
                  map.at("normal_area_z", index));
    Vector3 nRaster(map.at("normal_raster_x", index),
                    map.at("normal_raster_y", index),
                    map.at("normal_raster_z", index));
    map.at("directionalErrorArea", *iterator) =
        1.0f - static_cast<float>(std::abs(nA.dot(nArea)));
    map.at("directionalErrorRaster", *iterator) =
        1.0f - static_cast<float>(std::abs(nA.dot(nRaster)));
  }
  directionalErrorAreaSum =
      (directionalErrorAreaSum * 19.0 + map["directionalErrorArea"].sum()) /
      20.0;
  directionalErrorRasterSum =
      (directionalErrorRasterSum * 19.0 + map["directionalErrorRaster"].sum()) /
      20.0;
}

}  // namespace

int main(int argc, char** argv) {
  const int frames =
      std::atoi(grid_map::grid_map_demos::requireArg(argc, argv, 1, "20").c_str());
  const auto out_dir = std::filesystem::path(grid_map::grid_map_demos::requireArg(
      argc, argv, 2, "/tmp/grid_map_demos/normal_filter"));
  const auto chain_path = grid_map::grid_map_demos::resolveDemoPath(
      "config/normal_filter_comparison.yaml");

  filters::FilterChain<GridMap> filterChain;
  YAML::Node chain = YAML::LoadFile(chain_path.string());
  if (!filterChain.configure(chain, [](const std::string& type) {
        return FilterFactory::create(type);
      })) {
    AERROR << "Failed to configure normal filter chain";
    return 1;
  }

  GridMap map(
      {"elevation", "normal_analytic_x", "normal_analytic_y", "normal_analytic_z"});
  map.setFrameId("map");
  map.setGeometry(Length(2.5, 4.0), 0.02, Position(0.0, 0.0));
  const Size gridMapSize = map.getSize();

  constexpr double noise_on_map = 0.015;
  constexpr double outliers_percentage = 0.0;
  constexpr double surfaceSpeed = 0.5;
  constexpr double surfaceSlope = 0.2;
  constexpr double surfaceBias = -0.04;
  constexpr double wavePeriod = 5.0;

  double directionalErrorAreaSum = 0.0;
  double directionalErrorRasterSum = 0.0;

  for (int frame = 0; frame < frames; ++frame) {
    const double t = frame / 10.0;
    for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) = static_cast<float>(
          surfaceBias +
          surfaceSlope * std::sin(surfaceSpeed * t + wavePeriod * position.y()) *
              position.x());
      Vector3 normalAnalytic(
          -surfaceSlope * std::sin(surfaceSpeed * t + wavePeriod * position.y()),
          -position.x() * std::cos(surfaceSpeed * t + wavePeriod * position.y()),
          1.0);
      normalAnalytic.normalize();
      map.at("normal_analytic_x", *it) = normalAnalytic.x();
      map.at("normal_analytic_y", *it) = normalAnalytic.y();
      map.at("normal_analytic_z", *it) = normalAnalytic.z();
    }
    map.add("elevation_filtered", map.get("elevation"));
    if (noise_on_map != 0.0) {
      mapAddNoise(map, gridMapSize, noise_on_map);
    }
    if (outliers_percentage != 0.0) {
      mapAddOutliers(map, gridMapSize, outliers_percentage);
    }
    if (noise_on_map != 0.0 || outliers_percentage != 0.0) {
      mapAverageFiltering(map, gridMapSize, 0.1);
    }
    if (!filterChain.update(map, map)) {
      AERROR << "Filter chain update failed";
      return 1;
    }
    normalsErrorCalculation(map, gridMapSize, directionalErrorAreaSum,
                            directionalErrorRasterSum);
  }

  AINFO << "Avg directionalErrorArea=" << directionalErrorAreaSum
        << " directionalErrorRaster=" << directionalErrorRasterSum;
  grid_map::grid_map_demos::saveLayerAsPng(map, "elevation_filtered",
                                 out_dir / "elevation_filtered.png");
  grid_map::grid_map_demos::saveLayerAsPng(map, "directionalErrorArea",
                                 out_dir / "error_area.png");
  grid_map::grid_map_demos::saveLayerAsPng(map, "directionalErrorRaster",
                                 out_dir / "error_raster.png");
  grid_map::grid_map_demos::saveMapProto(map, out_dir / "map.pb");
  return 0;
}

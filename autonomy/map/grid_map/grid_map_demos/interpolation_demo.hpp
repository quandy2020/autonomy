/*
 * InterpolationDemo.hpp
 *
 *  Created on: Mar 16, 2020
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_core/type_defs.hpp"

#include <chrono>
#include <functional>
#include <map>
#include <string>

namespace grid_map {
namespace grid_map_demos {

/*
 * This demo visualizes the quality of function approximation
 * using different interpolation algorithms. It creates an analytical function,
 * which is sampled at discrete points. These discrete
 * values are then used for interpolation in order to recover
 * an approximation of the original function. The interpolation
 * result is visualized and error measures are computed and
 * printed in the console.
 *
 * The parameters are located in the "grid_map_demos/config/interpolation_demo.yaml"
 * file.
 *
 * For more info refer to README.md in the grid_map folder.
 *
 * For more details on the algorithms used,
 * refer to file: CubicInterpolation.hpp in the grid_map_core
 * package.
 *
 * Options for which interpolation algorithm to use are defined in
 * grid_map_core/include/grid_map_core/TypeDefs.hpp file.
 *
 * Interpolation is meant to be used with method "atPosition" defined in GridMap.hpp file.
 * To see how to use different interpolation methods in your own code,
 * take a look at definition of method "interpolateInputMap" in InterpolationDemo.cpp file.
 *
 */

/*
 * Different analytical functions used
 * to evaluate the quality of interpolation approximation.
 */
enum class Worlds: int {
  Poly,
  GaussMixture,
  Tanh,
  Sine,
  NUM_WORLDS
};

static const std::string demoLayer = "demo";

struct AnalyticalFunctions
{
  std::function<double(double, double)> f_;
};

struct Error {
  double meanSquare_ = 0.0;
  double meanAbs_ = 0.0;
  double max_ = 0.0;
};

/*
 * Create map geometry and allocate memory
 */
grid_map::GridMap createMap(const grid_map::Length &length, double resolution,
                            const grid_map::Position &pos);

/*
 * Fill grid map with values computed from analytical function
 */
void fillGridMap(grid_map::GridMap *map, const AnalyticalFunctions &functions);

/*
 * Create various analytical functions
 */
AnalyticalFunctions createPolyWorld(grid_map::GridMap *map);
AnalyticalFunctions createSineWorld(grid_map::GridMap *map);
AnalyticalFunctions createTanhWorld(grid_map::GridMap *map);
AnalyticalFunctions createGaussianWorld(grid_map::GridMap *map);


/*
 * Create high resolution grid map, used for visualizing the ground truth,
 * and create sparse grid map with data points that are used for the
 * interpolation.
 */
AnalyticalFunctions createWorld(Worlds world, double highResolution, double lowResolution,
                                double length, double width,
                                grid_map::GridMap *groundTruthHighRes,
                                grid_map::GridMap *groundTruthLowRes);

/*
 * Allocate memory and set geometry for the interpolated grid map
 */
grid_map::GridMap createInterpolatedMapFromDataMap(const grid_map::GridMap &dataMap, double desiredResolution);

/*
 * Compute the interpolated values
 */
void interpolateInputMap(const grid_map::GridMap &dataMap,
                                      grid_map::InterpolationMethods interpolationMethod, grid_map::GridMap *inteprolatedMap);

/*
 * Compute error measures between the ground truth and the interpolated map
 */
Error computeInterpolationError(const AnalyticalFunctions &groundTruth,
                                          const grid_map::GridMap &map);

static const std::map<std::string, Worlds> worlds = {
    { "Sine",  Worlds::Sine },
    { "Poly",  Worlds::Poly },
    { "Gauss", Worlds::GaussMixture },
    { "Tanh",  Worlds::Tanh } };

static const std::map<std::string, grid_map::InterpolationMethods> interpolationMethods = {
    { "Nearest",              grid_map::InterpolationMethods::INTER_NEAREST},
    { "Linear",               grid_map::InterpolationMethods::INTER_LINEAR },
    { "Cubic_convolution",    grid_map::InterpolationMethods::INTER_CUBIC_CONVOLUTION },
    { "Cubic",                grid_map::InterpolationMethods::INTER_CUBIC } };

/*
 * Class that actually runs the demo
 * publishes maps for visualization
 * computes errors
 * measures times required for computation
 */
class InterpolationDemo
{

 public:
  struct Options {
    std::string world = "Sine";
    std::string interpolationMethod = "Nearest";
    double groundTruthResolution = 0.02;
    double dataResolution = 0.1;
    double interpolatedResolution = 0.02;
    double worldWidth = 4.0;
    double worldLength = 4.0;
    std::string outDir = "/tmp/grid_map_demos/interpolation";
  };

  explicit InterpolationDemo(const Options& options);

  const grid_map::GridMap& groundTruthMap() const { return groundTruthMap_; }
  const grid_map::GridMap& dataSparseMap() const { return dataSparseMap_; }
  const grid_map::GridMap& interpolatedMap() const { return interpolatedMap_; }

 private:
  using clk = std::chrono::steady_clock;
  using ErrorAndDuration = std::pair<Error, double>;
  struct Statistic {
    Error error_;
    std::string world_;
    std::string interpolationMethod_;
    double duration_ = 0.0;
  };
  using Statistics = std::map<std::string, std::map<std::string, Statistic> >;

  void runDemo();
  ErrorAndDuration interpolateAndComputeError(const std::string world, const std::string &method) const;
  Statistics computeStatistics() const;
  void printStatistics(const Statistics &stats) const;

  grid_map::GridMap dataSparseMap_;
  grid_map::GridMap groundTruthMap_;
  grid_map::GridMap interpolatedMap_;

  std::string world_;
  std::string interpolationMethod_;
  double groundTruthResolution_ = 0.02;
  double dataResolution_ = 0.1;
  double interpolatedResolution_ = 0.02;
  AnalyticalFunctions groundTruth_;
  double worldWidth_ = 4.0;
  double worldLength_ = 4.0;
  std::string outDir_;

};

}  // namespace grid_map_demos
}  // namespace grid_map

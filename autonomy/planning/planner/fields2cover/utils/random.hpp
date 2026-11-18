/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file random.hpp
 * @brief Fields2Cover random.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <time.h>
#include <limits>
#include <random>
#include <boost/math/constants/constants.hpp>
#include "autonomy/planning/planner/fields2cover/types/point.hpp"
#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/linear_ring.hpp"
#include "autonomy/planning/planner/fields2cover/types/field.hpp"
#include "autonomy/planning/planner/fields2cover/types/cell.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {


/**
 * @class autonomy::planning::planner::fields2cover::Random
 * @brief Seeded PRNG helpers for tests and synthetic field generation.
 */
class Random {
 public:
  /**
   * @brief Seed the generator so experiments are reproducible.
   */
  explicit Random(uint32_t seed = static_cast<uint32_t>(time(NULL)));

  /**
   * @cond DOXYGEN_SHOULD_SKIP_THIS
   */
  ~Random();
  Random(const Random&);
  Random &operator=(const Random&);
  Random(Random&&);
  Random &operator=(Random&&);
  /**
   * @endcond
   */

 public:
  /**
   * @brief Get a random double \f$\in [0, 1]\f$.
   * @return Double \f$\in [0, 1]\f$
   */
  double GetRandomDouble();

  /**
   * @brief Get a random double in a range with uniform probability.
   * @param[in] min Minimum of the range
   * @param[in] max Maximum of the range
   * @return Double \f$\in [min, max]\f$
   */
  double GetRandomLinear(double min, double max);

  /**
   * @brief Get a random double in a range with exponential probability.
   * Lower values are more probable than higher ones.
   * @param[in] min Minimum of the range
   * @param[in] max Maximum of the range
   * @return Double \f$\in [min, max]\f$
   */
  double GetRandomExp(double min, double max);

  /**
   * @brief Get a random double in a range [0;+inf) with exponential
   * probability.
   * Lower values are more probable than higher ones.
   * @param[in] lambda is the rate parameter
   * @return Double > 0
   */
  double GetRandomExpDist(double lambda);

  /**
   * @brief Return a random angle in radians.
   * @return random angle \f$ \in [0, 2\pi) \f$
   */
  double GetAngleRandom();

  /**
   * @brief Random cell generator used by tests.
   * @param[in] Area Target area of the generated cell.
   * @param[in] n_sides Number of sides.
   * @param[in] min_width Minimum distance from the center.
   * @param[in] max_width Maximum distance from the center.
   * @return Random cell with approximately @p Area.
   */
  autonomy::planning::planner::fields2cover::types::Cell GenerateRandCell(double Area, int n_sides,
      double min_width = 0.5, double max_width = 1.0);
  /**
   * @brief Random field Generator to make tests.
   * @param[in] Area Area of the Generated field
   * @param[in] n_sides Number of sides of the Generated field.
   * @param[in] min_width Minimum Distance to the center for the base field.
   * @param[in] max_width Maximum Distance to the center for the base field.
   * @return Field with Area Area an n_sides.
   */
  autonomy::planning::planner::fields2cover::types::Field GenerateRandField(double Area, int n_sides,
      double min_width = 0.5, double max_width = 1.0);

  autonomy::planning::planner::fields2cover::types::Cell GenConvexCell(double Area, size_t n_sides = 4);
  /**
   * @brief Generate a convex field.
   * @param[in] Area Area of the Generated field
   */
  autonomy::planning::planner::fields2cover::types::Field GenConvexField(double Area, size_t n_sides = 4);

  autonomy::planning::planner::fields2cover::types::Cell GenNonConvexCell(double Area);
  /**
   * @brief Generate a non-convex field.
   * @param[in] Area Area of the Generated field
   */
  autonomy::planning::planner::fields2cover::types::Field GenNonConvexField(double Area);

 private:
  std::mt19937 mt_;
};


}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy


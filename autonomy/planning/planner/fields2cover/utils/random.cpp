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

/**
 * @file random.cpp
 * @brief Fields2Cover random implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/utils/random.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {


Random::Random(uint32_t seed) : mt_(seed) {}
Random::~Random() = default;
Random::Random(const Random&) = default;
Random& Random::operator=(const Random&) = default;
Random::Random(Random&&) = default;
Random& Random::operator=(Random&&) = default;

double Random::GetRandomDouble() {
  return GetRandomLinear(0.0, 1.0);
}

double Random::GetRandomLinear(double min, double max) {
  assert(min < max);
  std::uniform_real_distribution<double> distribution(min, max);
  return distribution(mt_);
}

double Random::GetRandomExp(double min, double max) {
  assert(min < max);
  double log_min {log(min)};
  return exp((log(max) - log_min) * GetRandomDouble() + log_min);
}

double Random::GetRandomExpDist(double lambda) {
  std::exponential_distribution<double> distribution(lambda);
  return distribution(mt_);
}

double Random::GetAngleRandom() {
  return GetRandomLinear(0.0, boost::math::constants::two_pi<double>());
}

autonomy::planning::planner::fields2cover::types::Cell Random::GenerateRandCell(double Area, int n_sides,
    double min_width, double max_width) {
  if (n_sides <= 2) {
    throw std::invalid_argument("Minimum 3 sides needed");
  }
  if (Area <= 0) {
    throw std::invalid_argument("The Area should be positive");
  }
  autonomy::planning::planner::fields2cover::types::LinearRing border;
  for (size_t i = 0; i < n_sides; ++i) {
    double r = this->GetRandomLinear(min_width, max_width);
    double alpha = 2.0 * M_PI * (i + this->GetRandomDouble()) / n_sides;
    border.AddPoint(r * sin(alpha), r * cos(alpha));
  }
  border.AddPoint(border.StartPoint());
  autonomy::planning::planner::fields2cover::types::Cell cell {border};
  cell *= sqrt(Area / cell.Area());
  return cell;
}

autonomy::planning::planner::fields2cover::types::Field Random::GenerateRandField(double Area, int n_sides,
    double min_width, double max_width) {
  return autonomy::planning::planner::fields2cover::types::Field(autonomy::planning::planner::fields2cover::types::Cells(
      this->GenerateRandCell(Area, n_sides, min_width, max_width)));
}

autonomy::planning::planner::fields2cover::types::Cell Random::GenConvexCell(double Area, size_t n_sides) {
  autonomy::planning::planner::fields2cover::types::Cell cell;
  do {
    cell = this->GenerateRandCell(Area, n_sides);
  } while (!cell.IsConvex());
  return cell;
}

autonomy::planning::planner::fields2cover::types::Field Random::GenConvexField(double Area, size_t n_sides) {
  return autonomy::planning::planner::fields2cover::types::Field(autonomy::planning::planner::fields2cover::types::Cells(
        this->GenConvexCell(Area, n_sides)));
}

autonomy::planning::planner::fields2cover::types::Cell Random::GenNonConvexCell(double Area) {
  autonomy::planning::planner::fields2cover::types::Cell cell;
  do {
    cell = this->GenerateRandCell(Area, 9);
  } while (cell.IsConvex());
  return cell;
}

autonomy::planning::planner::fields2cover::types::Field Random::GenNonConvexField(double Area) {
  return autonomy::planning::planner::fields2cover::types::Field(autonomy::planning::planner::fields2cover::types::Cells(this->GenNonConvexCell(Area)));
}


}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy



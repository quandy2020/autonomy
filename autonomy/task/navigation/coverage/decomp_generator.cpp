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
 * @file decomp_generator.cpp
 * @brief Native complete-coverage decomp generator implementation.
 */

#include "autonomy/task/navigation/coverage/decomp_generator.hpp"

#include "autonomy/task/navigation/coverage/generator_utils.hpp"
namespace autonomy::task::coverage {
F2CCells DecompGenerator::Decompose(const F2CCells& cells,
                                    const proto::DecompMode& m) const {
  const auto mode =
      Unknown(m.mode()) ? Upper(config_.decomp_type) : Upper(m.mode());
  const double angle =
      m.has_split_angle() ? m.split_angle() : config_.decomp_split_angle;
  Angle(angle);
  if (mode == "NONE") return cells.Clone();
  std::unique_ptr<f2c::decomp::DecompositionBase> decomposition;
  if (mode == "TRAPEZOIDAL") {
    auto generator = std::make_unique<f2c::decomp::TrapezoidalDecomp>();
    generator->SetSplitAngle(angle);
    decomposition = std::move(generator);
  } else if (mode == "BOUSTROPHEDON") {
    auto generator = std::make_unique<f2c::decomp::BoustrophedonDecomp>();
    generator->SetSplitAngle(angle);
    decomposition = std::move(generator);
  }
  if (decomposition) {
    const auto rays =
        decomposition->GenSplitLines(cells, f2c::obj::DecompObjective());
    F2CMultiLineString interior;
    for (const auto& ray : rays) {
      if (ray.size() < 2 || ray.Length() < 1e-6) continue;
      const auto midpoint = (ray.StartPoint() + ray.EndPoint()) * 0.5;
      if (cells.IsPointIn(midpoint) &&
          cells.ClosestPointOnBorderTo(midpoint).Distance(midpoint) > 1e-6)
        interior.AddGeometry(ray.Clone());
    }
    // Boundary rays cannot divide a cell. Avoid degenerate line buffers that
    // produce numerical slivers when a convex cell needs no decomposition.
    if (interior.IsEmpty()) return cells.Clone();
    auto split = cells.SplitByLine(interior);
    F2CCells clean;
    for (const auto& cell : split) {
      if (cell.Area() > 1e-6) clean.AddGeometry(cell.simplify(1e-7));
    }
    if (clean.IsEmpty())
      throw RequestException("Decomposition produced no valid cells");
    return decomposition->merge(clean, f2c::obj::DecompObjective());
  }
  throw CoverageException(
      "Decomp mode must be NONE, TRAPEZOIDAL or BOUSTROPHEDON");
}

}  // namespace autonomy::task::coverage

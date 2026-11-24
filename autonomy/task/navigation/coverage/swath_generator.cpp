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
 * @file swath_generator.cpp
 * @brief Native complete-coverage swath generator implementation.
 */

#include "autonomy/task/navigation/coverage/swath_generator.hpp"

#include "autonomy/task/navigation/coverage/generator_utils.hpp"
namespace autonomy::task::coverage {
F2CSwathsByCells SwathGenerator::GenerateSwathsByCells(
    const F2CCells& cells, const proto::SwathMode& m) const {
  const bool defaults = Unknown(m.mode()) && Unknown(m.objective());
  if (!defaults && (Unknown(m.mode()) || Unknown(m.objective())))
    throw CoverageException("Swath override requires both mode and objective");
  const auto mode = Upper(defaults ? config_.swath_angle_type : m.mode());
  const auto objective = Upper(defaults ? config_.swath_type : m.objective());
  std::unique_ptr<f2c::obj::SGObjective> obj;
  if (objective == "LENGTH")
    obj = std::make_unique<f2c::obj::SwathLength>();
  else if (objective == "NUMBER")
    obj = std::make_unique<f2c::obj::NSwath>();
  else if (objective == "NUMBER_MODIFIED")
    obj = std::make_unique<f2c::obj::NSwathModified>();
  else if (objective == "COVERAGE")
    obj = std::make_unique<f2c::obj::FieldCoverage>();
  else
    throw CoverageException(
        "Swath objective must be LENGTH, NUMBER, NUMBER_MODIFIED or COVERAGE");
  f2c::sg::BruteForce gen;
  gen.SetAllowOverlap(config_.allow_overlap);
  if (mode == "BRUTE_FORCE") {
    const double step =
        defaults ? config_.step_angle
                 : (m.has_step_angle() ? m.step_angle() : config_.step_angle);
    Positive(step, "step_angle");
    if (step < 1e-4 || step > 2 * M_PI)
      throw RequestException("step_angle outside [1e-4,2*pi]");
    gen.SetStepAngle(step);
    return gen.GenerateBestSwaths(*obj, config_.operation_width, cells);
  }
  if (mode == "SET_ANGLE") {
    const double angle = defaults ? config_.swath_angle : m.best_angle();
    Angle(angle);
    return gen.GenerateSwaths(angle, config_.operation_width, cells);
  }
  throw CoverageException("Swath mode must be BRUTE_FORCE or SET_ANGLE");
}

}  // namespace autonomy::task::coverage

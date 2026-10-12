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

#include "bolza/sqp/sqp_settings.hpp"

#include <iostream>


#include <bolza/core/misc/load_data.hpp>

namespace bolza {
namespace sqp {

Settings LoadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << "\n #### Multiple-Shooting SQP Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::LoadPtreeValue(pt, settings.sqpIteration, fieldName + ".sqpIteration", verbose);
  loadData::LoadPtreeValue(pt, settings.deltaTol, fieldName + ".deltaTol", verbose);
  loadData::LoadPtreeValue(pt, settings.alpha_decay, fieldName + ".alpha_decay", verbose);
  loadData::LoadPtreeValue(pt, settings.alpha_min, fieldName + ".alpha_min", verbose);
  loadData::LoadPtreeValue(pt, settings.gamma_c, fieldName + ".gamma_c", verbose);
  loadData::LoadPtreeValue(pt, settings.g_max, fieldName + ".g_max", verbose);
  loadData::LoadPtreeValue(pt, settings.g_min, fieldName + ".g_min", verbose);
  loadData::LoadPtreeValue(pt, settings.armijoFactor, fieldName + ".armijoFactor", verbose);
  loadData::LoadPtreeValue(pt, settings.costTol, fieldName + ".costTol", verbose);
  loadData::LoadPtreeValue(pt, settings.dt, fieldName + ".dt", verbose);
  loadData::LoadPtreeValue(pt, settings.useFeedbackPolicy, fieldName + ".useFeedbackPolicy", verbose);
  loadData::LoadPtreeValue(pt, settings.createValueFunction, fieldName + ".createValueFunction", verbose);
  auto integratorName = sensitivity_integrator::ToString(settings.integratorType);
  loadData::LoadPtreeValue(pt, integratorName, fieldName + ".integratorType", verbose);
  settings.integratorType = sensitivity_integrator::FromString(integratorName);
  loadData::LoadPtreeValue(pt, settings.inequalityConstraintMu, fieldName + ".inequalityConstraintMu", verbose);
  loadData::LoadPtreeValue(pt, settings.inequalityConstraintDelta, fieldName + ".inequalityConstraintDelta", verbose);
  loadData::LoadPtreeValue(pt, settings.projectStateInputEqualityConstraints, fieldName + ".projectStateInputEqualityConstraints", verbose);
  loadData::LoadPtreeValue(pt, settings.extractProjectionMultiplier, fieldName + ".extractProjectionMultiplier", verbose);
  loadData::LoadPtreeValue(pt, settings.printSolverStatus, fieldName + ".printSolverStatus", verbose);
  loadData::LoadPtreeValue(pt, settings.printSolverStatistics, fieldName + ".printSolverStatistics", verbose);
  loadData::LoadPtreeValue(pt, settings.printLinesearch, fieldName + ".printLinesearch", verbose);
  loadData::LoadPtreeValue(pt, settings.enableLogging, fieldName + ".enableLogging", verbose);
  loadData::LoadPtreeValue(pt, settings.logSize, fieldName + ".logSize", verbose);
  loadData::LoadPtreeValue(pt, settings.logFilePath, fieldName + ".logFilePath", verbose);
  loadData::LoadPtreeValue(pt, settings.nThreads, fieldName + ".nThreads", verbose);
  loadData::LoadPtreeValue(pt, settings.threadPriority, fieldName + ".threadPriority", verbose);

  if (verbose) {
    std::cerr << settings.lqSettings;
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}
}  // namespace sqp
}  // namespace bolza
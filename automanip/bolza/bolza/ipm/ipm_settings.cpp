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

#include "bolza/ipm/ipm_settings.hpp"


#include <bolza/core/misc/load_data.hpp>

#include <iostream>
#include <stdexcept>

namespace bolza {
namespace ipm {

Settings LoadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << "\n #### Multiple-Shooting IPM Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::LoadPtreeValue(pt, settings.ipmIteration, fieldName + ".ipmIteration", verbose);
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
  loadData::LoadPtreeValue(pt, settings.computeLagrangeMultipliers, fieldName + ".computeLagrangeMultipliers", verbose);
  auto integratorName = sensitivity_integrator::ToString(settings.integratorType);
  loadData::LoadPtreeValue(pt, integratorName, fieldName + ".integratorType", verbose);
  settings.integratorType = sensitivity_integrator::FromString(integratorName);
  loadData::LoadPtreeValue(pt, settings.initialBarrierParameter, fieldName + ".initialBarrierParameter", verbose);
  loadData::LoadPtreeValue(pt, settings.targetBarrierParameter, fieldName + ".targetBarrierParameter", verbose);
  loadData::LoadPtreeValue(pt, settings.barrierReductionCostTol, fieldName + ".barrierReductionCostTol ", verbose);
  loadData::LoadPtreeValue(pt, settings.barrierReductionConstraintTol, fieldName + ".barrierReductionConstraintTol", verbose);
  loadData::LoadPtreeValue(pt, settings.barrierLinearDecreaseFactor, fieldName + ".barrierLinearDecreaseFactor", verbose);
  loadData::LoadPtreeValue(pt, settings.barrierSuperlinearDecreasePower, fieldName + ".barrierSuperlinearDecreasePower", verbose);
  loadData::LoadPtreeValue(pt, settings.fractionToBoundaryMargin, fieldName + ".fractionToBoundaryMargin", verbose);
  loadData::LoadPtreeValue(pt, settings.usePrimalStepSizeForDual, fieldName + ".usePrimalStepSizeForDual", verbose);
  loadData::LoadPtreeValue(pt, settings.initialSlackLowerBound, fieldName + ".initialSlackLowerBound", verbose);
  loadData::LoadPtreeValue(pt, settings.initialDualLowerBound, fieldName + ".initialDualLowerBound", verbose);
  loadData::LoadPtreeValue(pt, settings.initialSlackMarginRate, fieldName + ".initialSlackMarginRate", verbose);
  loadData::LoadPtreeValue(pt, settings.initialDualMarginRate, fieldName + ".initialDualMarginRate", verbose);
  loadData::LoadPtreeValue(pt, settings.printSolverStatus, fieldName + ".printSolverStatus", verbose);
  loadData::LoadPtreeValue(pt, settings.printSolverStatistics, fieldName + ".printSolverStatistics", verbose);
  loadData::LoadPtreeValue(pt, settings.printLinesearch, fieldName + ".printLinesearch", verbose);
  loadData::LoadPtreeValue(pt, settings.nThreads, fieldName + ".nThreads", verbose);
  loadData::LoadPtreeValue(pt, settings.threadPriority, fieldName + ".threadPriority", verbose);

  if (settings.initialSlackLowerBound <= 0.0) {
    throw std::runtime_error("[MultipleShootingIpmSettings] initialSlackLowerBound must be positive!");
  }
  if (settings.initialDualLowerBound <= 0.0) {
    throw std::runtime_error("[MultipleShootingIpmSettings] initialDualLowerBound must be positive!");
  }
  if (settings.initialSlackMarginRate < 0.0) {
    throw std::runtime_error("[MultipleShootingIpmSettings] initialSlackMarginRate must be non-negative!");
  }
  if (settings.initialDualMarginRate < 0.0) {
    throw std::runtime_error("[MultipleShootingIpmSettings] initialDualMarginRate must be non-negative!");
  }
  if (settings.fractionToBoundaryMargin <= 0.0 || settings.fractionToBoundaryMargin > 1.0) {
    throw std::runtime_error("[MultipleShootingIpmSettings] fractionToBoundaryMargin must be positive and no more than 1.0!");
  }

  if (verbose) {
    std::cerr << settings.lqSettings;
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}
}  // namespace ipm
}  // namespace bolza
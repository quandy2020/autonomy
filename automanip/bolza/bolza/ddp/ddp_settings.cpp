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

#include "bolza/ddp/ddp_settings.hpp"

#include <algorithm>
#include <iostream>
#include <unordered_map>


#include <bolza/core/misc/load_data.hpp>

namespace bolza {
namespace ddp {

std::string ToAlgorithmName(Algorithm type) {
  static const std::unordered_map<Algorithm, std::string> strategyMap{{Algorithm::SLQ, "SLQ"}, {Algorithm::ILQR, "ILQR"}};
  return strategyMap.at(type);
}

Algorithm FromAlgorithmName(std::string name) {
  static const std::unordered_map<std::string, Algorithm> strategyMap{{"SLQ", Algorithm::SLQ}, {"ILQR", Algorithm::ILQR}};
  std::transform(name.begin(), name.end(), name.begin(), ::toupper);
  return strategyMap.at(name);
}

Settings LoadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << "\n #### DDP Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }

  std::string algorithmName = ToAlgorithmName(settings.algorithm_);
  loadData::LoadPtreeValue(pt, algorithmName, fieldName + ".algorithm", verbose);
  settings.algorithm_ = FromAlgorithmName(algorithmName);

  loadData::LoadPtreeValue(pt, settings.nThreads_, fieldName + ".nThreads", verbose);
  loadData::LoadPtreeValue(pt, settings.threadPriority_, fieldName + ".threadPriority", verbose);

  loadData::LoadPtreeValue(pt, settings.maxNumIterations_, fieldName + ".maxNumIterations", verbose);
  loadData::LoadPtreeValue(pt, settings.minRelCost_, fieldName + ".minRelCost", verbose);
  loadData::LoadPtreeValue(pt, settings.constraintTolerance_, fieldName + ".constraintTolerance", verbose);

  loadData::LoadPtreeValue(pt, settings.displayInfo_, fieldName + ".displayInfo", verbose);
  loadData::LoadPtreeValue(pt, settings.displayShortSummary_, fieldName + ".displayShortSummary", verbose);
  loadData::LoadPtreeValue(pt, settings.checkNumericalStability_, fieldName + ".CheckNumericalStability", verbose);
  loadData::LoadPtreeValue(pt, settings.debugPrintRollout_, fieldName + ".debugPrintRollout", verbose);

  loadData::LoadPtreeValue(pt, settings.absTolODE_, fieldName + ".AbsTolODE", verbose);
  loadData::LoadPtreeValue(pt, settings.relTolODE_, fieldName + ".RelTolODE", verbose);
  loadData::LoadPtreeValue(pt, settings.maxNumStepsPerSecond_, fieldName + ".maxNumStepsPerSecond", verbose);
  loadData::LoadPtreeValue(pt, settings.timeStep_, fieldName + ".timeStep", verbose);
  auto integratorName = integrator_type::ToString(settings.backwardPassIntegratorType_);  // keep default
  loadData::LoadPtreeValue(pt, integratorName, fieldName + ".backwardPassIntegratorType", verbose);
  settings.backwardPassIntegratorType_ = integrator_type::FromString(integratorName);

  loadData::LoadPtreeValue(pt, settings.constraintPenaltyInitialValue_, fieldName + ".constraintPenaltyInitialValue", verbose);
  loadData::LoadPtreeValue(pt, settings.constraintPenaltyIncreaseRate_, fieldName + ".constraintPenaltyIncreaseRate", verbose);

  loadData::LoadPtreeValue(pt, settings.preComputeRiccatiTerms_, fieldName + ".preComputeRiccatiTerms", verbose);

  loadData::LoadPtreeValue(pt, settings.useFeedbackPolicy_, fieldName + ".useFeedbackPolicy", verbose);

  loadData::LoadPtreeValue(pt, settings.riskSensitiveCoeff_, fieldName + ".riskSensitiveCoeff", verbose);

  std::string strategyName = search_strategy::ToString(settings.strategy_);
  loadData::LoadPtreeValue(pt, strategyName, fieldName + ".strategy", verbose);
  settings.strategy_ = search_strategy::FromString(strategyName);

  switch (settings.strategy_) {
    case search_strategy::Type::LINE_SEARCH: {
      settings.lineSearch_ = line_search::Load(filename, fieldName + ".lineSearch", verbose);
      break;
    }
    case search_strategy::Type::LEVENBERG_MARQUARDT: {
      settings.levenbergMarquardt_ = levenberg_marquardt::Load(filename, fieldName + ".levenbergMarquardt", verbose);
      break;
    }
  }

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}

}  // namespace ddp
}  // namespace bolza

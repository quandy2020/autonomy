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

#include "bolza/ddp/search_strategy/strategy_settings.hpp"

#include <algorithm>
#include <unordered_map>


#include <bolza/core/misc/load_data.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace search_strategy {

std::string ToString(Type strategy) {
  static const std::unordered_map<Type, std::string> strategyMap{{Type::LINE_SEARCH, "LINE_SEARCH"},
                                                                 {Type::LEVENBERG_MARQUARDT, "LEVENBERG_MARQUARDT"}};
  return strategyMap.at(strategy);
}

Type FromString(std::string name) {
  static const std::unordered_map<std::string, Type> strategyMap{{"LINE_SEARCH", Type::LINE_SEARCH},
                                                                 {"LEVENBERG_MARQUARDT", Type::LEVENBERG_MARQUARDT}};
  std::transform(name.begin(), name.end(), name.begin(), ::toupper);
  return strategyMap.at(name);
}

}  // namespace search_strategy

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace line_search {

Settings Load(const std::string& filename, const std::string& fieldName, bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);
  if (verbose) {
    std::cerr << " #### LINE_SEARCH Settings: {\n";
  }

  Settings settings;

  loadData::LoadPtreeValue(pt, settings.minStepLength, fieldName + ".minStepLength", verbose);
  loadData::LoadPtreeValue(pt, settings.maxStepLength, fieldName + ".maxStepLength", verbose);
  loadData::LoadPtreeValue(pt, settings.contractionRate, fieldName + ".contractionRate", verbose);
  loadData::LoadPtreeValue(pt, settings.armijoCoefficient, fieldName + ".armijoCoefficient", verbose);

  std::string hessianCorrectionStrategyName = hessian_correction::ToString(settings.hessianCorrectionStrategy);
  loadData::LoadPtreeValue(pt, hessianCorrectionStrategyName, fieldName + ".hessianCorrectionStrategy", verbose);
  settings.hessianCorrectionStrategy = hessian_correction::FromString(hessianCorrectionStrategyName);

  loadData::LoadPtreeValue(pt, settings.hessianCorrectionMultiple, fieldName + ".hessianCorrectionMultiple", verbose);

  if (verbose) {
    std::cerr << " #### }" << std::endl;
  }

  return settings;
}

}  // namespace line_search

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace levenberg_marquardt {

Settings Load(const std::string& filename, const std::string& fieldName, bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);
  if (verbose) {
    std::cerr << " #### LEVENBERG_MARQUARDT Settings: {\n";
  }

  Settings settings;

  loadData::LoadPtreeValue(pt, settings.minAcceptedPho, fieldName + ".minAcceptedPho", verbose);
  loadData::LoadPtreeValue(pt, settings.riccatiMultipleDefaultRatio, fieldName + ".riccatiMultipleDefaultRatio", verbose);
  loadData::LoadPtreeValue(pt, settings.riccatiMultipleDefaultFactor, fieldName + ".riccatiMultipleDefaultFactor", verbose);
  loadData::LoadPtreeValue(pt, settings.maxNumSuccessiveRejections, fieldName + ".maxNumSuccessiveRejections", verbose);
  if (verbose) {
    std::cerr << " #### }" << std::endl;
  }

  return settings;
}

}  // namespace levenberg_marquardt

}  // namespace bolza

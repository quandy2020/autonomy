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

#include "bolza/slp/pipg/pipg_settings.hpp"

#include <iostream>


#include <bolza/core/misc/load_data.hpp>

namespace bolza {
namespace pipg {

Settings LoadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << " #### PIPG Settings: {\n";
  }

  loadData::LoadPtreeValue(pt, settings.maxNumIterations, fieldName + ".maxNumIterations", verbose);
  loadData::LoadPtreeValue(pt, settings.absoluteTolerance, fieldName + ".absoluteTolerance", verbose);
  loadData::LoadPtreeValue(pt, settings.relativeTolerance, fieldName + ".relativeTolerance", verbose);

  loadData::LoadPtreeValue(pt, settings.lowerBoundH, fieldName + ".lowerBoundH", verbose);

  loadData::LoadPtreeValue(pt, settings.checkTerminationInterval, fieldName + ".checkTerminationInterval", verbose);
  loadData::LoadPtreeValue(pt, settings.displayShortSummary, fieldName + ".displayShortSummary", verbose);

  if (verbose) {
    std::cerr << " #### }\n";
  }

  return settings;
}

}  // namespace pipg
}  // namespace bolza
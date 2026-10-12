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

#include "examples/legged_robot/common/model_settings.hpp"


#include <bolza/core/misc/load_data.hpp>

namespace bolza {
namespace legged_robot {

ModelSettings LoadModelSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  ModelSettings modelSettings;

  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << "\n #### Legged Robot Model Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::LoadPtreeValue(pt, modelSettings.positionErrorGain, fieldName + ".positionErrorGain", verbose);
  loadData::LoadPtreeValue(pt, modelSettings.phaseTransitionStanceTime, fieldName + ".phaseTransitionStanceTime", verbose);

  loadData::LoadPtreeValue(pt, modelSettings.verboseCppAd, fieldName + ".verboseCppAd", verbose);
  loadData::LoadPtreeValue(pt, modelSettings.recompileLibrariesCppAd, fieldName + ".recompileLibrariesCppAd", verbose);
  loadData::LoadPtreeValue(pt, modelSettings.modelFolderCppAd, fieldName + ".modelFolderCppAd", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return modelSettings;
}

}  // namespace legged_robot
}  // namespace bolza

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

#include "bolza/oc/rollout/rollout_settings.hpp"


#include <bolza/core/misc/load_data.hpp>

namespace bolza {
namespace rollout {

Settings LoadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << "\n #### Rollout Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::LoadPtreeValue(pt, settings.absTolODE, fieldName + ".AbsTolODE", verbose);
  loadData::LoadPtreeValue(pt, settings.relTolODE, fieldName + ".RelTolODE", verbose);
  loadData::LoadPtreeValue(pt, settings.maxNumStepsPerSecond, fieldName + ".maxNumStepsPerSecond", verbose);
  loadData::LoadPtreeValue(pt, settings.timeStep, fieldName + ".timeStep", verbose);

  auto integratorName = integrator_type::ToString(settings.integratorType);  // keep default
  loadData::LoadPtreeValue(pt, integratorName, fieldName + ".integratorType", verbose);
  settings.integratorType = integrator_type::FromString(integratorName);

  loadData::LoadPtreeValue(pt, settings.checkNumericalStability, fieldName + ".checkNumericalStability", verbose);
  loadData::LoadPtreeValue(pt, settings.reconstructInputTrajectory, fieldName + ".reconstructInputTrajectory", verbose);

  auto rootFindingAlgorithmName = static_cast<int>(settings.rootFindingAlgorithm);  // keep default
  loadData::LoadPtreeValue(pt, rootFindingAlgorithmName, fieldName + ".rootFindingAlgorithm", verbose);
  settings.rootFindingAlgorithm = static_cast<RootFinderType>(rootFindingAlgorithmName);

  loadData::LoadPtreeValue(pt, settings.maxSingleEventIterations, fieldName + ".maxSingleEventIterations", verbose);
  loadData::LoadPtreeValue(pt, settings.useTrajectorySpreadingController, fieldName + ".useTrajectorySpreadingController", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}

}  // namespace rollout
}  // namespace bolza

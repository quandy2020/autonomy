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

#include "bolza/mpc/mpc_settings.hpp"


#include <bolza/core/misc/load_data.hpp>

namespace bolza {
namespace mpc {

Settings LoadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);

  Settings settings;

  if (verbose) {
    std::cerr << "\n #### MPC Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::LoadPtreeValue(pt, settings.timeHorizon_, fieldName + ".timeHorizon", verbose);
  loadData::LoadPtreeValue(pt, settings.solutionTimeWindow_, fieldName + ".solutionTimeWindow", verbose);
  loadData::LoadPtreeValue(pt, settings.coldStart_, fieldName + ".coldStart", verbose);

  loadData::LoadPtreeValue(pt, settings.debugPrint_, fieldName + ".debugPrint", verbose);

  loadData::LoadPtreeValue(pt, settings.mpcDesiredFrequency_, fieldName + ".mpcDesiredFrequency", verbose);
  loadData::LoadPtreeValue(pt, settings.mrtDesiredFrequency_, fieldName + ".mrtDesiredFrequency", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}

}  // namespace mpc
}  // namespace bolza

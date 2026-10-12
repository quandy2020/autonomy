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

#pragma once

#include <iostream>
#include <string>
#include <vector>


#include <bolza/core/types.hpp>
#include <bolza/core/misc/load_data.hpp>

namespace bolza {
namespace quadrotor {

struct QuadrotorParameters {
  // For safety, these parameters cannot be modified
  scalar_t quadrotorMass_ = 1.0;  // [kg]
  scalar_t Thzz_ = 1.0;
  scalar_t Thxxyy_ = 1.0;
  scalar_t gravity_ = 9.81;  // [m/s^2]
};

inline QuadrotorParameters LoadSettings(const std::string& filename, const std::string& fieldName = "QuadrotorParameters",
                                        bool verbose = true) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);

  QuadrotorParameters settings;

  if (verbose) {
    std::cerr << "\n #### Quadrotor Parameters:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::LoadPtreeValue(pt, settings.quadrotorMass_, fieldName + ".quadrotorMass", verbose);
  loadData::LoadPtreeValue(pt, settings.Thzz_, fieldName + ".Thzz", verbose);
  loadData::LoadPtreeValue(pt, settings.Thxxyy_, fieldName + ".Thxxyy", verbose);
  loadData::LoadPtreeValue(pt, settings.gravity_, fieldName + ".gravity", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return settings;
}

}  // namespace quadrotor
}  // namespace bolza

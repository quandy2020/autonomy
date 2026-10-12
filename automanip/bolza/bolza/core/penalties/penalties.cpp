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

#include "bolza/core/penalties/penalties.hpp"

#include <bolza/core/misc/load_data.hpp>


namespace bolza {
namespace loadData {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
void LoadPenaltyConfig<augmented::SmoothAbsolutePenalty::Config>(const std::string& fileName, const std::string& fieldName,
                                                                 augmented::SmoothAbsolutePenalty::Config& config, bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(fileName, pt);

  if (verbose) {
    std::cerr << "\n #### " << fieldName;
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::LoadPtreeValue(pt, config.scale, fieldName + ".scale", verbose);
  loadData::LoadPtreeValue(pt, config.relaxation, fieldName + ".relaxation", verbose);
  loadData::LoadPtreeValue(pt, config.stepSize, fieldName + ".stepSize", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
void LoadPenaltyConfig<augmented::QuadraticPenalty::Config>(const std::string& fileName, const std::string& fieldName,
                                                            augmented::QuadraticPenalty::Config& config, bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(fileName, pt);

  if (verbose) {
    std::cerr << "\n #### " << fieldName;
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::LoadPtreeValue(pt, config.scale, fieldName + ".scale", verbose);
  loadData::LoadPtreeValue(pt, config.stepSize, fieldName + ".stepSize", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
void LoadPenaltyConfig<augmented::ModifiedRelaxedBarrierPenalty::Config>(const std::string& fileName, const std::string& fieldName,
                                                                         augmented::ModifiedRelaxedBarrierPenalty::Config& config,
                                                                         bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(fileName, pt);

  if (verbose) {
    std::cerr << "\n #### " << fieldName;
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::LoadPtreeValue(pt, config.scale, fieldName + ".scale", verbose);
  loadData::LoadPtreeValue(pt, config.relaxation, fieldName + ".relaxation", verbose);
  loadData::LoadPtreeValue(pt, config.stepSize, fieldName + ".stepSize", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <>
void LoadPenaltyConfig<augmented::SlacknessSquaredHingePenalty::Config>(const std::string& fileName, const std::string& fieldName,
                                                                        augmented::SlacknessSquaredHingePenalty::Config& config,
                                                                        bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(fileName, pt);

  if (verbose) {
    std::cerr << "\n #### " << fieldName;
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::LoadPtreeValue(pt, config.scale, fieldName + ".scale", verbose);
  loadData::LoadPtreeValue(pt, config.stepSize, fieldName + ".stepSize", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n" << std::endl;
  }
}

}  // namespace loadData
}  // namespace bolza

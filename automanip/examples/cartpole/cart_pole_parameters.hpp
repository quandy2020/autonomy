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
namespace cartpole {

struct CartPoleParameters {
  /** Constructor */
  CartPoleParameters() { ComputeInertiaTerms(); }

  void Display() {
    std::cerr << "Cart-pole parameters: "
              << "\n";
    std::cerr << "cartMass:   " << cartMass_ << "\n";
    std::cerr << "poleMass:   " << poleMass_ << "\n";
    std::cerr << "poleLength: " << poleLength_ << "\n";
    std::cerr << "poleMoi:    " << poleMoi_ << "\n";
    std::cerr << "maxInput:   " << maxInput_ << "\n";
    std::cerr << "gravity:    " << gravity_ << "\n";
  }

  /** Loads the Cart-Pole's parameters. */
  void LoadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true) {
    bolza::property_tree::ptree pt;
    bolza::property_tree::read_info(filename, pt);
    if (verbose) {
      std::cerr << "\n #### Cart-pole Parameters:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::LoadPtreeValue(pt, cartMass_, fieldName + ".cartMass", verbose);
    loadData::LoadPtreeValue(pt, poleMass_, fieldName + ".poleMass", verbose);
    loadData::LoadPtreeValue(pt, poleLength_, fieldName + ".poleLength", verbose);
    loadData::LoadPtreeValue(pt, maxInput_, fieldName + ".maxInput", verbose);
    loadData::LoadPtreeValue(pt, gravity_, fieldName + ".gravity", verbose);
    ComputeInertiaTerms();
    if (verbose) {
      std::cerr << " #### =============================================================================\n" << std::endl;
    }
  }

  scalar_t cartMass_ = 1.0;       // [kg]
  scalar_t poleMass_ = 1.0;       // [kg]
  scalar_t poleLength_ = 1.0;     // [m]
  scalar_t maxInput_ = 6.0;       // [N]
  scalar_t gravity_ = 9.8;        // [m/s^2]
  scalar_t poleHalfLength_ = -1;  // [m]
  scalar_t poleMoi_ = -1;         // [kg*m^2]
  scalar_t poleSteinerMoi_ = -1;  // [kg*m^2]

 private:
  void ComputeInertiaTerms() {
    poleHalfLength_ = poleLength_ / 2.0;
    poleMoi_ = 1.0 / 12.0 * poleMass_ * (poleLength_ * poleLength_);
    poleSteinerMoi_ = poleMoi_ + poleMass_ * (poleHalfLength_ * poleHalfLength_);
  }
};

}  // namespace cartpole
}  // namespace bolza

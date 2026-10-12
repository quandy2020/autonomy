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

#include <array>
#include <cppad/cg.hpp>
#include <iostream>
#include <memory>

#include <bolza/pinocchio/centroidal/access_helper_functions.hpp>
#include <bolza/tools/common/rotation_transforms.hpp>

#include "examples/legged_robot/common/types.hpp"

namespace bolza {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Counts contact feet */
inline size_t NumberOfClosedContacts(const contact_flag_t& contactFlags) {
  size_t numStanceLegs = 0;
  for (auto legInContact : contactFlags) {
    if (legInContact) {
      ++numStanceLegs;
    }
  }
  return numStanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/** Computes an input with zero joint velocity and forces which equally distribute the robot weight between contact feet. */
inline vector_t WeightCompensatingInput(const CentroidalModelInfoTpl<scalar_t>& info, const contact_flag_t& contactFlags) {
  const auto numStanceLegs = NumberOfClosedContacts(contactFlags);
  vector_t input = vector_t::Zero(info.inputDim);
  if (numStanceLegs > 0) {
    const scalar_t totalWeight = info.robotMass * 9.81;
    const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
    for (size_t i = 0; i < contactFlags.size(); i++) {
      if (contactFlags[i]) {
        centroidal_model::GetContactForces(input, i, info) = forceInInertialFrame;
      }
    }  // end of i loop
  }
  return input;
}

}  // namespace legged_robot
}  // namespace bolza

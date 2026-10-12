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

#include <memory>

#include "examples/mobile_manipulator/mobile_manipulator_pre_computation.hpp"
#include <bolza/pinocchio/self_collision/self_collision_constraint.hpp>

namespace bolza {
namespace mobile_manipulator {

class MobileManipulatorSelfCollisionConstraint final : public SelfCollisionConstraint {
 public:
  MobileManipulatorSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping,
                                           PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
      : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance) {}
  ~MobileManipulatorSelfCollisionConstraint() override = default;
  MobileManipulatorSelfCollisionConstraint(const MobileManipulatorSelfCollisionConstraint& other) = default;
  MobileManipulatorSelfCollisionConstraint* Clone() const { return new MobileManipulatorSelfCollisionConstraint(*this); }

  const PinocchioInterface& GetPinocchioInterface(const PreComputation& preComputation) const override {
    return cast<MobileManipulatorPreComputation>(preComputation).GetPinocchioInterface();
  }
};

}  // namespace mobile_manipulator
}  // namespace bolza

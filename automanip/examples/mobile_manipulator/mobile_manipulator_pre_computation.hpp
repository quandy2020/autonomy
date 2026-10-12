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
#include <string>

#include <bolza/core/pre_computation.hpp>
#include <bolza/pinocchio/interface/pinocchio_interface.hpp>

#include "examples/mobile_manipulator/manipulator_model_info.hpp"
#include "examples/mobile_manipulator/mobile_manipulator_pinocchio_mapping.hpp"

namespace bolza {
namespace mobile_manipulator {

/** Callback for caching and reference update */
class MobileManipulatorPreComputation : public PreComputation {
 public:
  MobileManipulatorPreComputation(PinocchioInterface pinocchioInterface, const ManipulatorModelInfo& info);

  ~MobileManipulatorPreComputation() override = default;

  MobileManipulatorPreComputation(const MobileManipulatorPreComputation& rhs) = delete;
  MobileManipulatorPreComputation* Clone() const override;

  void Request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;
  void RequestFinal(RequestSet request, scalar_t t, const vector_t& x) override;

  PinocchioInterface& GetPinocchioInterface() { return pinocchioInterface_; }
  const PinocchioInterface& GetPinocchioInterface() const { return pinocchioInterface_; }

 private:
  PinocchioInterface pinocchioInterface_;
  MobileManipulatorPinocchioMapping pinocchioMapping_;
};

}  // namespace mobile_manipulator
}  // namespace bolza

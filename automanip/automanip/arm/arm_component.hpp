/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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

/**
 * @file arm_component.hpp
 * @brief Autolink TimerComponent wrapping ArmManager (DAG / mainboard).
 */

#ifndef AUTOMANIP_ARM_ARM_COMPONENT_HPP_
#define AUTOMANIP_ARM_ARM_COMPONENT_HPP_

#include "automanip/arm/arm_manager.hpp"
#include "autolink/component/timer_component.hpp"

namespace automanip {
namespace arm {

/**
 * @brief DAG class_name: "automanip::arm::ArmComponent".
 */
class ArmComponent : public autolink::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;

 protected:
  void Clear() override;

 private:
  ArmManager manager_;
};

}  // namespace arm
}  // namespace automanip

#endif  // AUTOMANIP_ARM_ARM_COMPONENT_HPP_

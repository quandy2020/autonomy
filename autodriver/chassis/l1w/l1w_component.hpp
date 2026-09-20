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

/**
 * @file l1w_component.hpp
 * @brief Autolink TimerComponent for GENISOM 钢镚 L1-W (DAG / mainboard).
 *
 * class_name: autodriver::chassis::l1w::L1wComponent
 * module_library: libautodriver_l1w.so
 */

#ifndef AUTODRIVER_CHASSIS_L1W_L1W_COMPONENT_HPP_
#define AUTODRIVER_CHASSIS_L1W_L1W_COMPONENT_HPP_

#include "chassis/chassis_manager.hpp"
#include "autolink/component/timer_component.hpp"

namespace autodriver {
namespace chassis {
namespace l1w {

/**
 * @class autodriver::chassis::l1w::L1wComponent
 * @brief Parameterized L1-W chassis component (full HighLevel surface).
 *
 * YAML: config/chassis/l1w.yaml (host / gait / tools / simulate …).
 */
class L1wComponent : public autolink::TimerComponent {
public:
  bool Init() override;
  bool Proc() override;

protected:
  void Clear() override;

private:
  ChassisManager manager_;
};

}  // namespace l1w
}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_L1W_L1W_COMPONENT_HPP_

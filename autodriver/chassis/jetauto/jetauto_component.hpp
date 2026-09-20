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
 * @file jetauto_component.hpp
 * @brief Autolink TimerComponent for Hiwonder JetAuto chassis (DAG / mainboard).
 *
 * class_name: autodriver::chassis::jetauto::JetAutoComponent
 * module_library: libautodriver_jetauto.so
 * config_file_path: YAML under AUTODRIVER_PATH/config/ (e.g. chassis/jetauto.yaml)
 */

#ifndef AUTODRIVER_CHASSIS_JETAUTO_JETAUTO_COMPONENT_HPP_
#define AUTODRIVER_CHASSIS_JETAUTO_JETAUTO_COMPONENT_HPP_

#include "chassis/chassis_manager.hpp"
#include "autolink/component/timer_component.hpp"

namespace autodriver {
namespace chassis {
namespace jetauto {

/**
 * @class autodriver::chassis::jetauto::JetAutoComponent
 * @brief Parameterized JetAuto chassis component.
 *
 * Loads YAML via TimerComponentConfig.config_file_path, forces
 * chassis.backend=jetauto, then starts ChassisManager. Tunables live in that
 * YAML (port/baud/drive_mode/wheel geometry — see config/chassis/jetauto.yaml).
 */
class JetAutoComponent : public autolink::TimerComponent {
public:
  bool Init() override;
  bool Proc() override;

protected:
  void Clear() override;

private:
  ChassisManager manager_;
};

}  // namespace jetauto
}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_JETAUTO_JETAUTO_COMPONENT_HPP_

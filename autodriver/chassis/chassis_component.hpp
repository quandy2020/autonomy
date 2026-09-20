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
 * @file chassis_component.hpp
 * @brief Autolink TimerComponent wrapping ChassisManager (DAG / mainboard).
 */

#ifndef AUTODRIVER_CHASSIS_CHASSIS_COMPONENT_HPP_
#define AUTODRIVER_CHASSIS_CHASSIS_COMPONENT_HPP_

#include "chassis/chassis_manager.hpp"
#include "autolink/component/timer_component.hpp"

namespace autodriver {
namespace chassis {

/**
 * @class autodriver::chassis::ChassisComponent
 * @brief Loads autodriver YAML and starts ChassisManager on Init().
 *
 * DAG timer_components class_name: "autodriver::chassis::ChassisComponent"
 * config_file_path: YAML with chassis: block (e.g. config/chassis/jetauto.yaml).
 * Proc() is a heartbeat; Manager owns cmd_vel / state publish threads.
 */
class ChassisComponent : public autolink::TimerComponent {
public:
  bool Init() override;
  bool Proc() override;

protected:
  void Clear() override;

private:
  ChassisManager manager_;
};

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_CHASSIS_COMPONENT_HPP_

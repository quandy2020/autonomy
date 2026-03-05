/*
 * Copyright 2025 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/vehicle/vehicle.hpp"

namespace autonomy {
namespace vehicle {

using ::autonomy::vehicle::proto::KinematicsControlCommand;
using ::autonomy::vehicle::proto::VehicleInfo;
using ::autonomy::vehicle::proto::VehicleModel;

Vehicle::Vehicle(const std::string& name) : name_(name) {}

bool Vehicle::Initialize(const VehicleModel& model) {
  model_ = model;
  initialized_ = true;

  // 用模型中的身份信息初始化 info
  info_.set_vehicle_type(model_.vehicle_type());
  info_.set_vehicle_id(model_.vehicle_id());
  info_.set_vehicle_name(model_.vehicle_name());

  return true;
}

bool Vehicle::GetVehicleInfo(VehicleInfo* info) {
  if (!info) {
    return false;
  }
  if (!initialized_) {
    return false;
  }

  *info = info_;
  return true;
}

bool Vehicle::ApplyCommand(const KinematicsControlCommand& command) {
  if (!initialized_) {
    return false;
  }

  // 缓存最近一次控制指令
  last_command_ = command;

  // 将期望的速度 / 加速度直接反映到 info 中，便于上层观察
  if (command.has_velocity()) {
    *info_.mutable_velocity() = command.velocity();
  }
  if (command.has_acceleration()) {
    *info_.mutable_acceleration() = command.acceleration();
  }

  // TODO(duyongquan): 在此处接入真实硬件 / 仿真底层，将 command 下发执行

  return true;
}

}  // namespace vehicle
}  // namespace autonomy

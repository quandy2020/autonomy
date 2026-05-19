/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include "autonomy/vehicle/vehicle_server.hpp"

namespace autonomy {
namespace vehicle {

using ::autonomy::vehicle::proto::KinematicsControlCommand;
using ::autonomy::vehicle::proto::Vehicle;
using ::autonomy::vehicle::proto::VehicleInfo;
using ::autonomy::vehicle::proto::VehicleModel;

VehicleServer::VehicleServer(const VehicleModel& model,
                             VehicleInterface::SharedPtr iface)
    : interface_(std::move(iface)), kinematics_(model) {
    // 初始化内部 Vehicle 消息中的模型配置
    *vehicle_.mutable_model() = model;
}

VehicleServer::~VehicleServer() = default;

bool VehicleServer::UpdateInfoFromInterface() {
    if (!interface_) {
        return false;
    }

    VehicleInfo info;
    if (!interface_->GetVehicleInfo(&info)) {
        return false;
    }

    *vehicle_.mutable_info() = info;
    return true;
}

void VehicleServer::SetCommand(const KinematicsControlCommand& command) {
    if (!interface_) {
        return;
    }

    // 先复制一份指令，并基于 VehicleModel 进行限幅
    KinematicsControlCommand limited_cmd = command;
    kinematics_.ApplyLimits(&limited_cmd);

    // 缓存到内部 Vehicle 消息中
    *vehicle_.mutable_command() = limited_cmd;

    // 下发到底层接口
    interface_->ApplyCommand(limited_cmd);

}

void VehicleServer::HandleVehicleMessage(
    const ::autonomy::vehicle::proto::Vehicle& vehicle_msg) {
    // 简单地覆盖当前缓存，后续可在此处增加一致性 / 权限检查
    vehicle_ = vehicle_msg;
}

}  // namespace vehicle
}  // namespace autonomy

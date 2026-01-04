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

    // 创建 autolink 节点，用于在图中发布 / 订阅 Vehicle 消息
    node_ = ::autolink::CreateNode("vehicle_server", "");
    if (node_) {
        // 订阅外部 Vehicle 消息（例如远程监控 / 仿真）
        vehicle_reader_ =
            node_->CreateReader<::autonomy::vehicle::proto::Vehicle>(
                "vehicle",
                [this](
                    const std::shared_ptr<::autonomy::vehicle::proto::Vehicle>&
                        msg) {
                    if (msg) {
                        this->HandleVehicleMessage(*msg);
                    }
                });

        // 发布本地聚合的 Vehicle 状态
        vehicle_writer_ =
            node_->CreateWriter<::autonomy::vehicle::proto::Vehicle>("vehicle");
    }
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

    // 将最新状态发布到 autolink 网络（如果有 writer）
    if (vehicle_writer_) {
        vehicle_writer_->Write(vehicle_);
    }

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

    // 将更新后的 Vehicle 消息发布出去
    if (vehicle_writer_) {
        vehicle_writer_->Write(vehicle_);
    }
}

void VehicleServer::HandleVehicleMessage(
    const ::autonomy::vehicle::proto::Vehicle& vehicle_msg) {
    // 简单地覆盖当前缓存，后续可在此处增加一致性 / 权限检查
    vehicle_ = vehicle_msg;
}

}  // namespace vehicle
}  // namespace autonomy

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

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/vehicle/common/vehicle_inteface.hpp"
#include "autonomy/vehicle/motion/kinematics_control.hpp"
#include "autonomy/vehicle/proto/vehicle_options.pb.h"

namespace autonomy {
namespace vehicle {

/**
 * @brief VehicleServer 负责在上层应用与底层 VehicleInterface 之间做桥接：
 *
 * - 持有 VehicleModel（模型配置）、VehicleInfo（运行时状态）和当前的
 *   KinematicsControlCommand（控制指令）
 * - 使用 KinematicsControl 对控制指令进行动力学限幅
 * - 将限幅后的指令下发给具体的 VehicleInterface 实现
 * - 从 VehicleInterface 周期性读取 VehicleInfo，并对外提供统一访问接口
 *
 * 该类本身不直接依赖具体硬件平台，通过 VehicleInterface 抽象适配差速车、
 * 阿克曼车、UGV/UAV 等不同类型的移动机器人。
 */
class VehicleServer
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(VehicleServer);

    /**
     * @brief 构造函数
     * @param model  车辆 / 机器人模型配置参数
     * @param iface  底层 VehicleInterface 实现（差速 / 阿克曼 / UAV 等）
     */
    VehicleServer(const ::autonomy::vehicle::proto::VehicleModel& model,
                  VehicleInterface::SharedPtr iface);

    /**
     * @brief 析构函数
     */
    ~VehicleServer();

    /**
     * @brief 从底层接口更新一次 VehicleInfo
     * @return true 成功，false 失败
     */
    bool UpdateInfoFromInterface();

    /**
     * @brief 设置（并下发）新的运动学控制指令
     *
     * - 先通过 KinematicsControl 进行限幅
     * - 再将指令缓存到内部 Vehicle 消息中
     * - 最后调用 VehicleInterface::ApplyCommand 下发到底层
     */
    void SetCommand(
        const ::autonomy::vehicle::proto::KinematicsControlCommand& command);

    /**
     * @brief 获取当前聚合的 Vehicle proto（包含 header + model + info +
     * command）
     */
    const ::autonomy::vehicle::proto::Vehicle& vehicle() const {
        return vehicle_;
    }

    /**
     * @brief 只读访问当前模型配置
     */
    const ::autonomy::vehicle::proto::VehicleModel& model() const {
        return vehicle_.model();
    }

    /**
     * @brief 只读访问当前运行时状态
     */
    const ::autonomy::vehicle::proto::VehicleInfo& info() const {
        return vehicle_.info();
    }

    /**
     * @brief 只读访问当前控制指令
     */
    const ::autonomy::vehicle::proto::KinematicsControlCommand& command()
        const {
        return vehicle_.command();
    }

private:
    // 顶层 Vehicle 聚合消息（header + model + info + command）
    ::autonomy::vehicle::proto::Vehicle vehicle_;

    // 底层硬件 / 仿真接口
    VehicleInterface::SharedPtr interface_;

    // 运动学限幅工具
    motion::KinematicsControl kinematics_;

    /**
     * @brief 处理从外部收到的 Vehicle 消息（例如来自远程监控 / 仿真）
     *        当前实现简单地用外部消息覆盖内部缓存，后续可在此处做权限/一致性检查。
     */
    void HandleVehicleMessage(
        const ::autonomy::vehicle::proto::Vehicle& vehicle);
};

}  // namespace vehicle
}  // namespace autonomy

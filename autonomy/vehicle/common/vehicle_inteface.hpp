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
#include "autonomy/vehicle/proto/vehicle_options.pb.h"

namespace autonomy {
namespace vehicle {

/**
 * @brief 车辆 / 移动机器人底层接口抽象
 *
 * 该接口负责对接具体硬件或仿真底层，实现：
 * - 基于 VehicleModel 进行初始化（几何参数、动力学约束等）
 * - 定期从底层读取状态，填充 VehicleInfo
 * - 接收上层的 KinematicsControlCommand，并下发到底层执行
 *
 * 上层模块通常通过 VehicleServer 持有一个 VehicleInterface::SharedPtr，
 * 而具体平台（差速底盘、阿克曼小车、无人机等）各自实现该接口。
 */
class VehicleInterface
{
public:
    /**
     * Define VehicleInterface::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(VehicleInterface);

    /**
     * @brief A constructor for autonomy::vehicle::VehicleInterface
     */
    explicit VehicleInterface();

    /**
     * @brief Virtual destructor
     */
    virtual ~VehicleInterface() = default;

    /**
     * @brief 使用车辆 / 机器人模型参数初始化底层接口
     * @param model VehicleModel proto，包含尺寸和动力学约束
     * @return true 初始化成功，false 失败
     */
    virtual bool Initialize(const ::autonomy::vehicle::proto::VehicleModel& model) = 0;

    /**
     * @brief 从底层读取当前车辆 / 机器人状态
     * @param info 输出参数，填充为当前 VehicleInfo
     * @return true 读取成功，false 失败
     */
    virtual bool GetVehicleInfo(::autonomy::vehicle::proto::VehicleInfo* info) = 0;

    /**
     * @brief 向底层发送运动学控制指令
     * @param command 上层规划 / 控制模块给出的 KinematicsControlCommand
     * @return true 发送成功，false 失败
     */
    virtual bool ApplyCommand(const ::autonomy::vehicle::proto::KinematicsControlCommand& command) = 0;
};

}  // namespace vehicle
}  // namespace autonomy
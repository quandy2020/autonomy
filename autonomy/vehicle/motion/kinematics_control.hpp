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

#pragma once

#include "autonomy/vehicle/proto/vehicle_options.pb.h"

namespace autonomy {
namespace vehicle {
namespace motion {

/**
 * @brief 运动学控制工具类
 *
 * 主要职责：
 * - 基于 VehicleModel 中的动力学约束（最大速度/加速度/转向角等），对上层给出的
 *   KinematicsControlCommand 进行限幅（saturation）
 * - 将过大的线速度、角速度、加速度等裁剪到车辆/移动机器人模型所允许的范围内，
 *   避免上层规划指令直接越界到底层执行模块。
 *
 * 注意：
 * -
 * 这里只做“约束过滤”，不负责具体的轮速/舵角求解（那属于更底层的动力学/驱动层）。
 * - 对于不同运动学模型（差速、全向、阿克曼等），这里统一按 Twist 的 x / y / z
 *   和 yaw 角速度来做限幅，具体分解由 VehicleInterface 的具体实现完成。
 */
class KinematicsControl
{
public:
    /**
     * @brief 构造函数
     * @param model 车辆 / 机器人的模型参数（包含速度、加速度、转向等上限）
     */
    explicit KinematicsControl(
        const ::autonomy::vehicle::proto::VehicleModel& model);

    /**
     * @brief 对给定的运动学控制指令进行限幅（就地修改）
     *
     * - 线速度：根据 max_linear_speed / max_reverse_speed 进行裁剪
     * - 角速度：根据 max_angular_speed 进行裁剪
     * - 线加速度 / 角加速度：根据 max_linear_acceleration /
     * max_angular_acceleration 进行裁剪
     *
     * @param cmd 待限幅的控制指令（非空指针）
     */
    void ApplyLimits(
        ::autonomy::vehicle::proto::KinematicsControlCommand* cmd) const;

private:
    ::autonomy::vehicle::proto::VehicleModel model_;
};

}  // namespace motion
}  // namespace vehicle
}  // namespace autonomy

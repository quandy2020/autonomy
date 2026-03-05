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

#include <string>

#include "autonomy/common/macros.hpp"
#include "autonomy/vehicle/common/vehicle_inteface.hpp"
#include "autonomy/vehicle/proto/vehicle_options.pb.h"

namespace autonomy {
namespace vehicle {

/**
 * @brief 默认的 VehicleInterface 实现，用于在进程内桥接上层与底层
 *
 * 设计目标：
 * - 作为一个通用的、与具体硬件无关的 VehicleInterface 实现，
 *   提供对 VehicleModel / VehicleInfo / KinematicsControlCommand 的基本管理
 * - 上层可以通过 VehicleServer 或直接持有 Vehicle::SharedPtr 与之交互
 * - 未来可在 ApplyCommand() 和 GetVehicleInfo() 中接入真实硬件 / 仿真接口
 *
 * 当前实现：
 * - Initialize()：缓存模型参数，并用其初始化基础 VehicleInfo 字段
 * - GetVehicleInfo()：返回内部缓存的 VehicleInfo（后续可扩展为真正从底层读取）
 * - ApplyCommand()：暂时只缓存最近一次控制指令，并在 info 中更新速度 /
 * 加速度等字段 以便上层可以看到最新期望值；实际硬件下发逻辑留作 TODO。
 */
class Vehicle : public VehicleInterface {
 public:
  /**
   * Define Vehicle::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(Vehicle);

  /**
   * @brief 构造函数
   * @param name 逻辑车辆名称（便于调试 / 日志）
   */
  explicit Vehicle(const std::string& name = "vehicle");

  /**
   * @brief 使用车辆 / 机器人模型参数初始化底层接口
   */
  bool Initialize(const ::autonomy::vehicle::proto::VehicleModel& model) override;

  /**
   * @brief 从内部缓存或底层读取当前车辆 / 机器人状态
   */
  bool GetVehicleInfo(::autonomy::vehicle::proto::VehicleInfo* info) override;

  /**
   * @brief 向底层发送运动学控制指令（当前仅缓存并更新 info）
   */
  bool ApplyCommand(const ::autonomy::vehicle::proto::KinematicsControlCommand& command) override;

 private:
  // 顶层 Vehicle 聚合消息（header + model + info + command）

  std::string name_;

  // 模型配置（尺寸、动力学约束等）
  ::autonomy::vehicle::proto::VehicleModel model_;

  // 运行时状态缓存（用于上层查询）
  ::autonomy::vehicle::proto::VehicleInfo info_;

  // 最近一次接收到的控制指令
  ::autonomy::vehicle::proto::KinematicsControlCommand last_command_;

  bool initialized_{false};
};

}  // namespace vehicle
}  // namespace autonomy

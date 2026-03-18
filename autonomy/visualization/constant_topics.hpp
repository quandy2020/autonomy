/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

namespace autonomy {
namespace visualization {

/**
 * @brief 定义常用的 Foxglove topic 名称常量
 *
 * 这些常量用于统一管理 topic 名称，避免硬编码字符串。
 * 所有 topic 都使用 geometry_msgs::PoseStamped 消息类型。
 */
namespace Topics {
// 目标位姿：用于设置导航目标点
constexpr const char* GOAL_POSE = "/goal_pose";

// 点击点：用于处理用户在 3D 场景中的点击事件
// 支持常见的 topic 名称格式
constexpr const char* CLICKED_POINT = "/clicked_point";
constexpr const char* CLICK_POINT = "/click_point";  // 备用名称

// 初始位姿：用于设置机器人的初始位置
// 支持常见的 topic 名称格式
constexpr const char* INITIAL_POSE = "/initialpose";
constexpr const char* INIT_POSE = "/init_pose";  // 备用名称
}  // namespace Topics

}  // namespace visualization
}  // namespace autonomy

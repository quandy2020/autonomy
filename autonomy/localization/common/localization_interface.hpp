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

#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/localization/proto/localization_options.pb.h"

namespace autonomy {
namespace localization {
namespace common {

/**
 * @brief 定位算法接口
 *
 * 所有定位算法（如 AMCL）都应该实现此接口
 */
class LocalizationInterface
{
public:
    /**
     * @brief 定义智能指针类型
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(LocalizationInterface)

    /**
     * @brief 构造函数
     */
    LocalizationInterface() = default;

    /**
     * @brief 析构函数
     */
    virtual ~LocalizationInterface() = default;

    /**
     * @brief 启动定位算法（配置并激活）
     * @param options 定位选项配置
     * @param name 算法名称
     * @return true 成功，false 失败
     */
    virtual bool Start(const proto::LocalizationOptions& options,
                       const std::string& name) = 0;

    /**
     * @brief 停止定位算法（停用、清理并关闭）
     * @return true 成功，false 失败
     */
    virtual bool Stop() = 0;

    /**
     * @brief 设置初始位姿
     * @param pose 初始位姿（带协方差）
     * @return true 成功，false 失败
     */
    virtual bool SetInitialPose(
        const commsgs::geometry_msgs::PoseWithCovariance& pose) = 0;

    /**
     * @brief 获取当前位姿估计
     * @param pose 输出的位姿（带协方差）
     * @return true 成功，false 失败
     */
    virtual bool GetPose(commsgs::geometry_msgs::PoseWithCovariance& pose) = 0;
};

proto::LocalizationOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace common
}  // namespace localization
}  // namespace autonomy
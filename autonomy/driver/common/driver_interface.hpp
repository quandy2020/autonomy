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

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/common/port.hpp"
#include "autonomy/driver/proto/driver_options.pb.h"
#include "autonomy/sensor/data.hpp"

namespace autonomy {
namespace driver {
namespace common {

/**
 * @class DriverInterface
 * @brief 驱动器接口基类，定义不同类型驱动器的通用接口
 *
 * DriverInterface 为不同类型的传感器驱动器（如
 * LidarDriver、ImuDriver、CameraDriver 等）
 * 提供统一的接口，支持插件化的驱动器实现。
 */
class DriverInterface
{
public:
    /**
     * Define DriverInterface::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(DriverInterface)

    /**
     * @brief 构造函数
     */
    DriverInterface() = default;

    /**
     * @brief 析构函数
     */
    virtual ~DriverInterface() = default;

    DriverInterface(const DriverInterface&) = delete;
    DriverInterface& operator=(const DriverInterface&) = delete;

    /**
     * @brief 配置驱动器
     * @param name 驱动器名称
     * @param options 驱动器配置选项（通用选项，子类可转换为具体类型）
     * @return true 成功，false 失败
     */
    virtual bool Configure(const std::string& name,
                           const proto::DriverOptions& options) = 0;

    /**
     * @brief 初始化驱动器
     * @return true 成功，false 失败
     */
    virtual bool Initialize() = 0;

    /**
     * @brief 启动驱动器
     */
    virtual void Start() = 0;

    /**
     * @brief 停止驱动器
     */
    virtual void Stop() = 0;

    /**
     * @brief 清理资源
     */
    virtual void Cleanup() = 0;

    /**
     * @brief 获取驱动器名称
     * @return 驱动器名称
     */
    virtual std::string GetName() const = 0;

    /**
     * @brief 获取传感器ID列表
     * @return 传感器ID列表
     */
    virtual std::vector<std::string> GetSensorIds() const = 0;

    /**
     * @brief 检查传感器是否已注册
     * @param sensor_id 传感器ID
     * @return true 已注册，false 未注册
     */
    virtual bool IsSensorRegistered(const std::string& sensor_id) const = 0;

    /**
     * @brief 注册传感器数据处理器
     * @param sensor_id 传感器ID
     * @param handler 数据处理器函数
     * @return true 成功，false 失败
     */
    virtual bool RegisterSensorHandler(
        const std::string& sensor_id,
        std::function<void(const std::string&,
                           const std::shared_ptr<sensor::Data>&)>
            handler) = 0;

    /**
     * @brief 取消注册传感器数据处理器
     * @param sensor_id 传感器ID
     */
    virtual void UnregisterSensorHandler(const std::string& sensor_id) = 0;
};

/**
 * @brief 从 Lua 参数字典加载驱动器选项
 * @param parameter_dictionary Lua 参数字典
 * @return 驱动器选项
 */
proto::DriverOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

/**
 * @brief 从配置文件创建驱动器选项
 * @param configuration_directory 配置目录
 * @param configuration_basename 配置文件名
 * @return 驱动器选项
 */
proto::DriverOptions CreateOptions(const std::string& configuration_directory,
                                   const std::string& configuration_basename);

}  // namespace common
}  // namespace driver
}  // namespace autonomy

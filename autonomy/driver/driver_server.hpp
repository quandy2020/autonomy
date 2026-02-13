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

#include <memory>
#include <mutex>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/driver/proto/driver_options.pb.h"

namespace autonomy {
namespace driver {

// 前向声明（在 cpp 文件中包含完整定义）
class DriverEngine;
class DataRouter;

/**
 * @class DriverServer
 * @brief 驱动服务器类，统一管理传感器驱动的订阅、处理和转发
 *
 * DriverServer 负责：
 * 1. 管理 DriverEngine（传感器数据订阅和处理）
 * 2. 管理 DataRouter（数据路由，支持 ROS2 和内部驱动）
 * 3. 提供统一的启动、停止接口
 * 4. 支持从配置文件加载配置
 */
class DriverServer
{
public:
    /**
     * Define DriverServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(DriverServer)

    /**
     * @brief 构造函数
     * @param options 驱动配置选项
     */
    explicit DriverServer(const proto::DriverOptions& options);

    /**
     * @brief 析构函数
     */
    ~DriverServer();

    DriverServer(const DriverServer&) = delete;
    DriverServer& operator=(const DriverServer&) = delete;

    /**
     * @brief 初始化驱动服务器
     * @return true 成功，false 失败
     */
    bool Initialize();

    /**
     * @brief 启动驱动服务器
     */
    void Start();

    /**
     * @brief 停止驱动服务器
     */
    void Stop();

    /**
     * @brief 等待关闭（阻塞直到服务停止）
     */
    void WaitForShutdown();

    /**
     * @brief 获取驱动引擎
     * @return DriverEngine 的共享指针
     */
    std::shared_ptr<DriverEngine> GetDriverEngine() const {
        return driver_engine_;
    }

    /**
     * @brief 获取数据路由器
     * @return DataRouter 的共享指针
     */
    std::shared_ptr<DataRouter> GetDataRouter() const {
        return data_router_;
    }

protected:
    // Autolink 节点（使用 shared_ptr 管理生命周期）
    std::shared_ptr<::autolink::Node> node_ptr_{nullptr};

    // Autolink 节点指针（用于传递给其他组件）
    ::autolink::Node* node_{nullptr};

    // 驱动引擎（管理传感器订阅和处理）
    std::shared_ptr<DriverEngine> driver_engine_{nullptr};

    // 数据路由器（管理数据路由）
    std::shared_ptr<DataRouter> data_router_{nullptr};

    // 配置选项
    proto::DriverOptions options_;

    // 状态标志
    bool initialized_{false};
    bool started_{false};

    // 互斥锁（用于保护状态标志）
    mutable std::mutex mutex_;
};

}  // namespace driver
}  // namespace autonomy
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

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "autolink/autolink.hpp"
#include "autolink/time/time.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/localization/common/localization_interface.hpp"
#include "autonomy/localization/proto/localization_options.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/transform_broadcaster.hpp"

namespace autonomy {
namespace localization {

/**
 * @brief LocalizationServer - 定位服务器
 *
 * 提供定位服务的对外接口。
 * 支持多种定位算法（如 AMCL）的加载、配置和运行。
 * 接收静态地图 /map，根据 initial_pose 设置初始位置。
 * 以 30Hz 实时发布 TF 变换链：map -> odom -> base_link -> base_footprint
 * LocalizationServer 负责 map -> odom 变换
 */
class LocalizationServer
{
public:
    /**
     * @brief 定义智能指针类型
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(LocalizationServer)

    /**
     * @brief 构造函数
     * @param options 定位选项配置
     * @param node_name 可选的节点名称，如果为空则使用默认名称
     */
    explicit LocalizationServer(const proto::LocalizationOptions& options,
                                const std::string& node_name = "localization_server");

    /**
     * @brief 析构函数
     */
    ~LocalizationServer();

    /**
     * @brief 启动定位服务器（配置并激活）
     * @return true 成功，false 失败
     */
    bool Start();

    /**
     * @brief 停止定位服务器（停用、清理并关闭）
     * @return true 成功，false 失败
     */
    bool Stop();

private:
};

}  // namespace localization
}  // namespace autonomy

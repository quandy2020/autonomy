/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/autolink.hpp"
#include "autolink/timer/timer.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/map/proto/map_options.pb.h"
#include "autonomy/map/utils/data_loader_utils.hpp"

namespace autonomy {
namespace map {

/**
 * @class autonomy::map::MapServer
 * @brief 地图服务类，负责加载和发布静态地图
 *
 * MapServer 负责：
 * 1. 从文件加载静态地图
 * 2. 通过 autolink 发布静态地图
 */
class MapServer
{
public:
    /**
     * Define MapServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(MapServer)

    /**
     * @brief 构造函数
     * @param options 地图服务配置选项
     * @param node_name 可选的节点名称，如果为空则使用默认名称
     */
    MapServer(const proto::MapOptions& options,
              const std::string& node_name = "");

    /**
     * @brief 析构函数
     */
    ~MapServer();

    MapServer(const MapServer&) = delete;
    MapServer& operator=(const MapServer&) = delete;

    /**
     * @brief 启动地图服务
     */
    void Start();

    /**
     * @brief 关闭地图服务（调用 Stop 并等待关闭完成）
     */
    void Shutdown();

    /**
     * @brief 获取原始静态地图数据
     * @param static_map 输出的静态地图数据
     * @return 是否成功获取
     */
    bool GetRawStaticMap(commsgs::map_msgs::OccupancyGrid& static_map) const;

    /**
     * @brief 获取静态地图文件路径
     * @return 静态地图文件路径
     */
    std::string GetStaticMapFile() const;

    /**
     * @brief 获取静态地图名称
     * @return 静态地图名称
     */
    std::string GetStaticMapName() const {
        return static_map_name_;
    }

protected:
    // 原始地图名称（静态地图，从文件加载或SLAM提供）
    std::string static_map_name_;

    // 配置选项
    proto::MapOptions options_;

    // autolink 节点
    std::unique_ptr<::autolink::Node> node_{nullptr};
    std::shared_ptr<autolink::Writer<commsgs::map_msgs::OccupancyGrid>>
        static_map_writer_{nullptr};
    std::shared_ptr<::autolink::Timer> static_map_timer_{nullptr};

    // map message
    commsgs::map_msgs::OccupancyGrid::SharedPtr static_map_msg_{nullptr};

    std::atomic<bool> running_{false};
    std::mutex publish_mutex_;
};

}  // namespace map
}  // namespace autonomy

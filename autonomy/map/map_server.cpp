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

#include "autonomy/map/map_server.hpp"

#include <unistd.h>  // for getpid()

#include "autolink/common/log.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/map/constants.hpp"
#include "autonomy/map/costmap_2d/map_io.hpp"
#include "autonomy/map/utils/data_loader_utils.hpp"

namespace autonomy {
namespace map {

MapServer::MapServer(const proto::MapOptions& options, const std::string& node_name) : options_{options} {
    // 创建 autolink 节点
    std::string actual_node_name = node_name.empty() ? kMapServerNodeName : node_name;
    // 添加进程ID以确保节点名唯一（避免共享内存冲突）
    actual_node_name += "_" + std::to_string(getpid());
    node_ = ::autolink::CreateNode(actual_node_name, "");

    AINFO << "MapServer created with node name: " << actual_node_name;

    // 检查 map_file 是否有效
    bool has_static_map = !options_.map_file().empty();
    if (has_static_map) {
        // 静态地图名称
        static_map_name_ = options_.map_name().empty() ? "map" : options_.map_name();

        AINFO << "Static map configuration: map_file=" << options_.map_file() << ", map_name=" << static_map_name_;
    } else {
        AWARN << "Static map file is not configured.";
    }

    // 创建静态地图发布器
    std::string static_map_topic = options_.map_topic().empty() ? "map" : options_.map_topic();
    static_map_writer_ = node_->CreateWriter<commsgs::map_msgs::OccupancyGrid>(static_map_topic);

    AINFO << "MapServer initialized successfully. Static map topic: " << static_map_topic;
}

MapServer::~MapServer() {
    if (running_.load()) {
        Shutdown();
    }
    AINFO << "MapServer destroyed.";
}

void MapServer::Start() {
    if (running_.load()) {
        AWARN << "MapServer is already running.";
        return;
    }

    running_.store(true);

    // 如果配置了发布频率 > 0，启动周期性发布定时器
    if (!static_map_name_.empty() && options_.publish_frequency() > 0.0) {
        double static_publish_freq = options_.publish_frequency();
        uint32_t period_ms = static_cast<uint32_t>(1000.0 / static_publish_freq);

        static_map_timer_ = std::make_shared<::autolink::Timer>(
            period_ms,
            [this]() {
                // 只在第一次读取地图，后续直接使用缓存
                if (!static_map_msg_) {
                    static_map_msg_ = std::make_shared<commsgs::map_msgs::OccupancyGrid>();
                    if (GetRawStaticMap(*static_map_msg_)) {
                        AINFO << "Static map loaded and cached.";
                    } else {
                        AWARN << "Failed to load static map.";
                        static_map_msg_.reset();
                        return;
                    }
                }

                // 使用缓存的地图消息进行发布
                if (static_map_msg_ && static_map_writer_) {
                    static_map_writer_->Write(*static_map_msg_);
                    AINFO << "Static map published.";
                }
            },
            false  // 周期性执行
        );
        static_map_timer_->Start();
        AINFO << "Static map publish timer started with frequency: " << static_publish_freq << " Hz (" << period_ms
              << " ms).";
    }

    AINFO << "MapServer started.";
}

void MapServer::Shutdown() {
    if (!running_.load()) {
        return;
    }

    running_.store(false);

    // 停止静态地图发布定时器
    if (static_map_timer_) {
        static_map_timer_->Stop();
        static_map_timer_.reset();
    }

    AINFO << "MapServer shutdown.";
}

bool MapServer::GetRawStaticMap(commsgs::map_msgs::OccupancyGrid& static_map) const {
    AINFO << "MapServer::Start: Publishing static map";

    std::string map_file = utils::GetMapDataFilesDirectory() + options_.map_file();

    if (map_file.empty()) {
        AWARN << "Static map file is not configured.";
        return false;
    }

    // 直接从文件加载原始地图数据（不经过 costmap 处理）
    if (::autonomy::map::costmap_2d::loadMapFromYaml(map_file, static_map) !=
        ::autonomy::map::costmap_2d::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
        AERROR << "Failed to load raw static map from file: " << map_file;
        return false;
    }

    // 设置 frame_id
    if (!options_.frame_id().empty()) {
        static_map.header.frame_id = options_.frame_id();
    } else {
        static_map.header.frame_id = "map";
    }

    return true;
}

std::string MapServer::GetStaticMapFile() const {
    return options_.map_file();
}

}  // namespace map
}  // namespace autonomy

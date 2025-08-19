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

#include "autonomy/tools/god_viewer/channel/channel_ccupancy_grid_hander.hpp"

#include <chrono>

#include "autonomy/map/costmap_2d/map_io.hpp"
#include "autonomy/map/costmap_2d/map_mode.hpp"
namespace autonomy {
namespace tools { 
namespace god_viewer { 
namespace channel {

OccupancyGridHandler::OccupancyGridHandler(ServerHander::SharedPtr options, const std::string& topic)
    : topic_{topic}
{
    LOG(INFO) << "Init path handler topic: " << topic_;
    channel_ = std::make_unique<foxglove::schemas::GridChannel>(
        foxglove::schemas::GridChannel::create(topic_).value());
}

bool OccupancyGridHandler::SendTest()
{
    std::string yaml_file = "/workspace/autonomy/configuration_files/map/turtlebot3_house.yaml";
    LOG(INFO) << "map_file: " << yaml_file;

    commsgs::map_msgs::OccupancyGrid map_data;
    if (map::costmap_2d::loadMapFromYaml(yaml_file, map_data) != map::costmap_2d::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
        LOG(ERROR) << "Load yaml file error.";
        return false;
    }
    return Send(map_data);
}

bool OccupancyGridHandler::Send(const commsgs::map_msgs::OccupancyGrid& msgs)
{
    auto grid = FromCommsgs(msgs);
    channel_->log(grid);
    return true;
}

foxglove::schemas::Grid OccupancyGridHandler::FromCommsgs(const commsgs::map_msgs::OccupancyGrid& msgs)
{
    foxglove::schemas::Grid grid;
    // 设置时间戳
    grid.timestamp = foxglove::schemas::Timestamp{msgs.header.stamp.sec, msgs.header.stamp.nanosec};
    
    // 设置坐标系
    grid.frame_id = msgs.header.frame_id;
    
    // 设置位姿 (原点在左下角)
    auto position = foxglove::schemas::Vector3{
        msgs.info.origin.position.x, 
        msgs.info.origin.position.y, 
        msgs.info.origin.position.z
    };
    auto orientation = foxglove::schemas::Quaternion{
        msgs.info.origin.orientation.x, 
        msgs.info.origin.orientation.y, 
        msgs.info.origin.orientation.z,
        msgs.info.origin.orientation.w
    };
    grid.pose = foxglove::schemas::Pose{
        position,
        orientation
    };

    // 设置网格尺寸
    grid.column_count = msgs.info.width;
    
    // 设置单元格大小
    grid.cell_size = foxglove::schemas::Vector2{msgs.info.resolution, msgs.info.resolution};
    
    // 设置数据布局 (行优先)
    grid.cell_stride = 1;  // 每个单元格1字节
    grid.row_stride = msgs.info.width * grid.cell_stride;  // 每行字节数

    // 设置字段 (占用值)
    grid.fields = {
        foxglove::schemas::PackedElementField{
            "intensity",  // 字段名
            0,            // offset
            foxglove::schemas::PackedElementField::NumericType::UINT8
        }
    };

    // 初始化地图数据 (0: 空闲, 100: 占用, 255: 未知)
    std::vector<std::byte> data(msgs.info.width * msgs.info.height);

    // 创建地图
    for (uint32_t y = 0; y < msgs.info.height; ++y) {
        for (uint32_t x = 0; x < msgs.info.width; ++x) {
            data[y * msgs.info.width + x] = static_cast<std::byte>(msgs.data[y * msgs.info.width + x]);
        }
    }
    grid.data = data;
    return grid;
}

}   // channel
}   // god_viewer
}   // namespace tools
}   // namespace autonomy
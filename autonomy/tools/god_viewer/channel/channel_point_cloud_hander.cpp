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

#include "autonomy/tools/god_viewer/channel/channel_point_cloud_hander.hpp"
#include "autonomy/map/costmap_3d/map_io.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer { 
namespace channel {

PointCloudHandler::PointCloudHandler(ServerHander::SharedPtr options, const std::string& topic)
    : topic_{topic}
{
    LOG(INFO) << "Init path handler topic: " << topic_;
    channel_ = std::make_unique<foxglove::schemas::PointCloudChannel>(
        foxglove::schemas::PointCloudChannel::create(topic_).value());
}

bool PointCloudHandler::SendTest()
{
    // std::string yaml_file = "/workspace/autonomy/configuration_files/map/turtlebot3_house.yaml";
    
    std::string ply_file = "/home/quandy/workspace/github/autonomy/src/autonomy/configuration_files/map/matterport_pointcloud.ply";
    LOG(INFO) << "ply_file: " << ply_file;

    commsgs::sensor_msgs::PointCloud map_data;
    map::costmap_3d::LoadPlyFile(ply_file, map_data);
    return Send(map_data);
}

bool PointCloudHandler::Send(const commsgs::sensor_msgs::PointCloud& msgs)
{
    auto grid = FromCommsgs(msgs);
    channel_->log(grid);
    return true;
}

foxglove::schemas::PointCloud PointCloudHandler::FromCommsgs(const commsgs::sensor_msgs::PointCloud& msgs)
{
    
    // Step 1: 转换头部信息
    foxglove::schemas::PointCloud data;
    data.timestamp = foxglove::schemas::Timestamp{msgs.header.stamp.sec, msgs.header.stamp.nanosec};
    // data.frame_id = msgs.header.frame_id;
    data.frame_id = "map";

    // Step 2: 设置点云结构
    const size_t num_points = msgs.points.size();
    const size_t num_channels = 4;

    // 计算每个点的字节长度 (x/y/z + 所有通道)
    const size_t point_stride = sizeof(float) * (3 + num_channels);
    data.point_stride = static_cast<uint32_t>(point_stride);
    
    // 调整数据缓冲区大小
    data.data.resize(num_points * point_stride);
        
    // Step 3: 构建字段描述
    // 添加坐标字段 (x, y, z)
    data.fields = {
        {"x", 0,                  foxglove::schemas::PackedElementField::NumericType::FLOAT32}, 
        {"y", sizeof(float),      foxglove::schemas::PackedElementField::NumericType::FLOAT32},
        {"z", 2 * sizeof(float),  foxglove::schemas::PackedElementField::NumericType::FLOAT32},
    };

    // Step 4: 填充点云数据
    // 使用指针操作提高性能
    // float* data_ptr = reinterpret_cast<float*>(msgs.points());
    std::byte* dst = data.data.data();
    for (size_t i = 0; i < num_points; ++i) {
        // 复制坐标 (x, y, z)
        const auto& pt = msgs.points[i];
        memcpy(dst, &pt.x, sizeof(float)); dst += sizeof(float);
        memcpy(dst, &pt.y, sizeof(float)); dst += sizeof(float);
        memcpy(dst, &pt.z, sizeof(float)); dst += sizeof(float);
    }
   
    return data;
}

}   // channel
}   // god_viewer
}   // namespace tools
}   // namespace autonomy
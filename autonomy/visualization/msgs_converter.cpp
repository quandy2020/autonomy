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

#include "autonomy/visualization/msgs_converter.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace visualization { 

foxglove::schemas::RawImage FromCommsgs(const commsgs::sensor_msgs::Image& msgs)
{
    foxglove::schemas::RawImage image;
    image.frame_id = "map";
    image.width = msgs.width;
    image.height = msgs.height;
    // image.encoding = msgs.encoding;

    LOG(INFO) << "msgs.encoding: " << msgs.encoding;
    image.encoding = "mono8";
    image.step = msgs.step;

    // 转换数据：从 std::vector<uint32_t> 到 std::vector<std::byte>
    size_t byte_count = image.width * image.height;
    image.data.resize(byte_count);
    
    // 将 uint32_t 向量中的数据复制回字节形式
    std::byte* raw_data = image.data.data();
    for (size_t i = 0; i < byte_count; i += sizeof(uint32_t)) {
        uint32_t value = msgs.data[i / sizeof(uint32_t)];
        size_t bytes_to_copy = std::min(sizeof(uint32_t), byte_count - i);
        
        // 将 uint32_t 复制回字节
        std::memcpy(raw_data + i, &value, bytes_to_copy);
    }

    return image;
}

foxglove::schemas::RawImage FromCommsgs(const cv::Mat& mat)
{
    foxglove::schemas::RawImage rawImage;
    rawImage.frame_id = "map";
    rawImage.width = static_cast<uint32_t>(mat.cols);
    rawImage.height = static_cast<uint32_t>(mat.rows);
    rawImage.step = static_cast<uint32_t>(mat.step);
    
    switch (mat.type()) {
        case CV_8UC1:
            rawImage.encoding = "mono8";
            break;
        case CV_8UC3:
            rawImage.encoding = "bgr8"; // OpenCV 默认使用 BGR 顺序
            break;
        case CV_8UC4:
            rawImage.encoding = "bgra8";
            break;
        case CV_16UC1:
            rawImage.encoding = "mono16";
            break;
        case CV_16UC3:
            rawImage.encoding = "bgr16";
            break;
        case CV_32FC1:
            rawImage.encoding = "32fc1";
            break;
        case CV_32FC3:
            rawImage.encoding = "32fc3";
            break;
        default:
            rawImage.encoding = "unknown";
            break;
    }
    
    size_t data_size = mat.total() * mat.elemSize();
    rawImage.data.resize(data_size);
    
    if (mat.isContinuous()) {
        std::memcpy(rawImage.data.data(), mat.data, data_size);
    } else {
        size_t offset = 0;
        for (int i = 0; i < mat.rows; ++i) {
            const uchar* row_data = mat.ptr<uchar>(i);
            std::memcpy(rawImage.data.data() + offset, row_data, mat.step);
            offset += mat.step;
        }
    }
    return rawImage;
}


foxglove::schemas::PointCloud FromCommsgs(const commsgs::sensor_msgs::PointCloud& msgs)
{
    foxglove::schemas::PointCloud pointcloud;
    return pointcloud;
}

foxglove::schemas::PointCloud FromCommsgs(const commsgs::sensor_msgs::PointCloud2& msgs)
{
    foxglove::schemas::PointCloud pointcloud;
    return pointcloud;
}

foxglove::schemas::LaserScan FromCommsgs(const commsgs::sensor_msgs::LaserScan& msgs)
{
    foxglove::schemas::LaserScan scan;

    // Timestamp of scan
    scan.timestamp = {msgs.header.stamp.sec, msgs.header.stamp.nanosec};

    // Frame of reference
    scan.frame_id = msgs.header.frame_id;

    // Bearing of first point, in radians
    scan.start_angle = msgs.angle_min;

    // Distance of detections from origin; assumed to be at equally-spaced angles between
    // `start_angle` and `end_angle`
    scan.end_angle = msgs.angle_max;

    // ranges
    scan.ranges.assign(msgs.ranges.begin(), msgs.ranges.end());
    
    // Intensity of detections
    for (auto intensity : msgs.intensities) {
        scan.intensities.push_back(intensity);
    }
    return scan;
}

foxglove::schemas::Point3 FromCommsgs(const commsgs::geometry_msgs::Point& msgs)
{
    return {
        msgs.x,
        msgs.y,
        msgs.z
    };
}

foxglove::schemas::Pose FromCommsgs(const commsgs::geometry_msgs::Pose& msgs)
{
    foxglove::schemas::Pose pose;
    // pose.position.x = static_cast<double>(msgs.position.x);
    // pose.position.y = msgs.pose.position.y;
    // pose.position.z = msgs.pose.position.z;
    // pose.orientation.x = msgs.pose.orientation.x;
    // pose.orientation.y = msgs.pose.orientation.y;
    // pose.orientation.z = msgs.pose.orientation.w;
    // pose.orientation.w = msgs.pose.orientation.w;
    return pose;

}

foxglove::schemas::LinePrimitive FromCommsgs(const commsgs::planning_msgs::Path& msgs)
{
    foxglove::schemas::LinePrimitive line;
    line.type = foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
    line.thickness = 0.08;
    line.color = foxglove::schemas::Color{0, 1, 0, 1};
    for (auto pose : msgs.poses) {
        line.points.push_back({
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        });
    }
    return line;
}

foxglove::schemas::FrameTransforms FromCommsgs(const commsgs::geometry_msgs::TransformStampeds& msgs)
{
    foxglove::schemas::FrameTransforms transforms;
    for (auto transform : msgs.transforms) {
        foxglove::schemas::FrameTransform tf;

        // Timestamp of transform
        tf.timestamp = {msgs.header.stamp.sec, msgs.header.stamp.nanosec};

        // Name of the parent frame
        tf.parent_frame_id = transform.header.frame_id;

        // Name of the child frame
        tf.child_frame_id = transform.child_frame_id;

        // Translation component of the transform
        tf.translation = {
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        };

        // Rotation component of the transform
        tf.rotation = {
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        };
        transforms.transforms.push_back(tf);
    }
    return transforms;
}

foxglove::schemas::LinePrimitive FromCommsgs(const commsgs::geometry_msgs::PolygonStamped& msgs)
{
    foxglove::schemas::LinePrimitive line;
    line.type = foxglove::schemas::LinePrimitive::LineType::LINE_LOOP;
    line.thickness = 0.08;
    line.color = foxglove::schemas::Color{0, 1, 0, 1};
    for (auto point : msgs.polygon.points) {
        line.points.push_back({
            point.x,
            point.y,
            point.z
        });
    }
    return line;
}

foxglove::schemas::Grid FromCommsgs(const commsgs::map_msgs::OccupancyGrid& msgs)
{
    foxglove::schemas::Grid grid;
    // 设置时间戳
    grid.timestamp = foxglove::schemas::Timestamp{msgs.header.stamp.sec, msgs.header.stamp.nanosec};
    
    // 设置坐标系
    grid.frame_id = msgs.header.frame_id;
    
    // 设置位姿 (原点在左下角)
    auto position = foxglove::schemas::Vector3{
        msgs.info.origin.position.x + msgs.info.resolution / 2.0,
        msgs.info.origin.position.y + msgs.info.resolution / 2.0,
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
            int8_t value = static_cast<int8_t>(msgs.data[y * msgs.info.width + x]);
            if (value == -1) {
                data[y * msgs.info.width + x] = static_cast<std::byte>(255);
            } else if (value >= 0 && value <= 100) {
                // 线性映射: 0-100 -> 0-255
                float scaled_value = static_cast<float>(value) * 255.0f / 100.0f;
                data[y * msgs.info.width + x] = static_cast<std::byte>(static_cast<uint8_t>(scaled_value));
            } else {
                data[y * msgs.info.width + x] = static_cast<std::byte>(128);
            }
        }
    }

    grid.data = data;
    return grid;
}

}   // namespace visualization
}   // namespace autonomy

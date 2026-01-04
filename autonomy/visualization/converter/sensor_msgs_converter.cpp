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

#include "autonomy/visualization/converter/sensor_msgs_converter.hpp"

#include <cmath>
#include <cstring>

#include <opencv2/opencv.hpp>

#include "autolink/autolink.hpp"
#include "autonomy/visualization/converter/converter_detail.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace impl {

using detail::CreateColor;
using detail::ExtractFrameId;
using detail::ExtractTimestamp;
using detail::SetEntityHeader;
using detail::SetPointCloudHeader;
using detail::SetRawImageHeader;

foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::sensor_msgs::LaserScan& message) {
    foxglove::schemas::SceneUpdate scene_update;

    if (message.ranges_size() == 0) {
        AWARN << "LaserScan has no ranges, returning empty SceneUpdate";
        return scene_update;
    }

    // 创建场景实体
    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    // 设置实体 ID
    entity.id = "laser_scan";
    entity.frame_locked = false;

    // 创建线条 primitive 来显示激光扫描
    foxglove::schemas::LinePrimitive line;
    line.type = foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
    line.thickness = 0.05;  // 5cm 线宽
    line.scale_invariant = false;

    // 设置颜色（红色）
    line.color = CreateColor(1.0, 0.0, 0.0, 1.0);

    // 将激光扫描数据转换为点
    float angle = message.angle_min();
    for (int i = 0; i < message.ranges_size(); ++i) {
        float range = message.ranges(i);

        // 过滤无效范围
        if (range >= message.range_min() && range <= message.range_max()) {
            foxglove::schemas::Point3 point;
            point.x = range * cos(angle);
            point.y = range * sin(angle);
            point.z = 0.0;
            line.points.push_back(point);
        }

        angle += message.angle_increment();
    }

    entity.lines.push_back(line);
    scene_update.entities.push_back(entity);

    AINFO << "Converted LaserScan with " << message.ranges_size()
          << " ranges to SceneUpdate";

    return scene_update;
}

foxglove::schemas::PointCloud ToFoxgloveImpl(
    const autonomy::commsgs::proto::sensor_msgs::PointCloud2& message) {
    foxglove::schemas::PointCloud pointcloud;

    if (message.width() == 0 || message.height() == 0) {
        AWARN << "PointCloud2 is empty, returning empty PointCloud";
        return pointcloud;
    }

    // 设置时间戳和 frame_id
    SetPointCloudHeader(pointcloud, ExtractTimestamp(message),
                        ExtractFrameId(message));

    // 转换 PointField 到 PackedElementField
    for (const auto& field : message.fields()) {
        foxglove::schemas::PackedElementField fg_field;
        fg_field.name = field.name();
        fg_field.offset = field.offset();

        // 转换数据类型 (ROS PointField datatype -> Foxglove NumericType)
        // INT8=1, UINT8=2, INT16=3, UINT16=4, INT32=5, UINT32=6, FLOAT32=7,
        // FLOAT64=8
        switch (field.datatype()) {
            case 1:  // INT8
                fg_field.type =
                    foxglove::schemas::PackedElementField::NumericType::INT8;
                break;
            case 2:  // UINT8
                fg_field.type =
                    foxglove::schemas::PackedElementField::NumericType::UINT8;
                break;
            case 3:  // INT16
                fg_field.type =
                    foxglove::schemas::PackedElementField::NumericType::INT16;
                break;
            case 4:  // UINT16
                fg_field.type =
                    foxglove::schemas::PackedElementField::NumericType::UINT16;
                break;
            case 5:  // INT32
                fg_field.type =
                    foxglove::schemas::PackedElementField::NumericType::INT32;
                break;
            case 6:  // UINT32
                fg_field.type =
                    foxglove::schemas::PackedElementField::NumericType::UINT32;
                break;
            case 7:  // FLOAT32
                fg_field.type =
                    foxglove::schemas::PackedElementField::NumericType::FLOAT32;
                break;
            case 8:  // FLOAT64
                fg_field.type =
                    foxglove::schemas::PackedElementField::NumericType::FLOAT64;
                break;
            default:
                AWARN << "Unknown PointField datatype: " << field.datatype()
                      << ", defaulting to FLOAT32";
                fg_field.type =
                    foxglove::schemas::PackedElementField::NumericType::FLOAT32;
                break;
        }
        pointcloud.fields.push_back(fg_field);
    }

    // 设置 point_stride
    pointcloud.point_stride = message.point_step();

    // 转换数据：repeated uint32 -> std::vector<std::byte>
    if (message.data_size() > 0) {
        pointcloud.data.reserve(message.data_size());
        for (int i = 0; i < message.data_size(); ++i) {
            pointcloud.data.push_back(
                static_cast<std::byte>(message.data(i) & 0xFF));
        }
    }

    AINFO << "Converted PointCloud2 to PointCloud: " << message.width() << "x"
          << message.height() << " points, " << pointcloud.fields.size()
          << " fields, " << pointcloud.data.size() << " bytes";

    return pointcloud;
}

foxglove::schemas::PointCloud ToFoxgloveImpl(
    const autonomy::commsgs::proto::sensor_msgs::PointCloud& message) {
    foxglove::schemas::PointCloud pointcloud;

    if (message.points_size() == 0) {
        AWARN << "PointCloud is empty, returning empty PointCloud";
        return pointcloud;
    }

    // 设置时间戳和 frame_id
    SetPointCloudHeader(pointcloud, ExtractTimestamp(message),
                        ExtractFrameId(message));

    // PointCloud 使用 PointXYZIT，需要转换为二进制格式
    constexpr size_t POINT_SIZE =
        16;  // 4 floats (x, y, z) + 1 uint32 (intensity) = 16 bytes

    // 定义字段
    foxglove::schemas::PackedElementField x_field;
    x_field.name = "x";
    x_field.offset = 0;
    x_field.type = foxglove::schemas::PackedElementField::NumericType::FLOAT32;
    pointcloud.fields.push_back(x_field);

    foxglove::schemas::PackedElementField y_field;
    y_field.name = "y";
    y_field.offset = 4;
    y_field.type = foxglove::schemas::PackedElementField::NumericType::FLOAT32;
    pointcloud.fields.push_back(y_field);

    foxglove::schemas::PackedElementField z_field;
    z_field.name = "z";
    z_field.offset = 8;
    z_field.type = foxglove::schemas::PackedElementField::NumericType::FLOAT32;
    pointcloud.fields.push_back(z_field);

    foxglove::schemas::PackedElementField intensity_field;
    intensity_field.name = "intensity";
    intensity_field.offset = 12;
    intensity_field.type =
        foxglove::schemas::PackedElementField::NumericType::UINT32;
    pointcloud.fields.push_back(intensity_field);

    // 设置 point_stride
    pointcloud.point_stride = POINT_SIZE;

    // 转换点数据
    pointcloud.data.reserve(message.points_size() * POINT_SIZE);
    for (const auto& point : message.points()) {
        // 写入 x, y, z (float32, 12 bytes)
        float x = point.x();
        float y = point.y();
        float z = point.z();
        const std::byte* x_bytes = reinterpret_cast<const std::byte*>(&x);
        const std::byte* y_bytes = reinterpret_cast<const std::byte*>(&y);
        const std::byte* z_bytes = reinterpret_cast<const std::byte*>(&z);
        pointcloud.data.insert(pointcloud.data.end(), x_bytes,
                               x_bytes + sizeof(float));
        pointcloud.data.insert(pointcloud.data.end(), y_bytes,
                               y_bytes + sizeof(float));
        pointcloud.data.insert(pointcloud.data.end(), z_bytes,
                               z_bytes + sizeof(float));

        // 写入 intensity (uint32, 4 bytes)
        uint32_t intensity = point.intensity();
        const std::byte* intensity_bytes =
            reinterpret_cast<const std::byte*>(&intensity);
        pointcloud.data.insert(pointcloud.data.end(), intensity_bytes,
                               intensity_bytes + sizeof(uint32_t));
    }

    AINFO << "Converted PointCloud to PointCloud: " << message.points_size()
          << " points, " << pointcloud.fields.size() << " fields, "
          << pointcloud.data.size() << " bytes";

    return pointcloud;
}

foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::sensor_msgs::Imu& message) {
    foxglove::schemas::SceneUpdate scene_update;

    // 创建场景实体
    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    entity.id = "imu";
    entity.frame_locked = false;

    // 创建箭头显示线性加速度
    if (message.has_linear_acceleration()) {
        foxglove::schemas::ArrowPrimitive accel_arrow;
        accel_arrow.pose = foxglove::schemas::Pose();
        accel_arrow.pose->position = foxglove::schemas::Vector3();
        accel_arrow.pose->position->x = 0.0;
        accel_arrow.pose->position->y = 0.0;
        accel_arrow.pose->position->z = 0.0;

        // 计算加速度向量的长度和方向
        double ax = message.linear_acceleration().x();
        double ay = message.linear_acceleration().y();
        double az = message.linear_acceleration().z();
        double length = sqrt(ax * ax + ay * ay + az * az);

        if (length > 0.001) {
            accel_arrow.shaft_length = length * 0.1;
            accel_arrow.head_length = accel_arrow.shaft_length * 0.2;
            accel_arrow.shaft_diameter = 0.02;
            accel_arrow.head_diameter = 0.05;

            accel_arrow.pose->orientation = foxglove::schemas::Quaternion();
            accel_arrow.pose->orientation->x = 0.0;
            accel_arrow.pose->orientation->y = 0.0;
            accel_arrow.pose->orientation->z = 0.0;
            accel_arrow.pose->orientation->w = 1.0;

            accel_arrow.color = CreateColor(0.0, 1.0, 0.0, 1.0);
            entity.arrows.push_back(accel_arrow);
        }
    }

    // 创建箭头显示角速度
    if (message.has_angular_velocity()) {
        foxglove::schemas::ArrowPrimitive angvel_arrow;
        angvel_arrow.pose = foxglove::schemas::Pose();
        angvel_arrow.pose->position = foxglove::schemas::Vector3();
        angvel_arrow.pose->position->x = 0.0;
        angvel_arrow.pose->position->y = 0.0;
        angvel_arrow.pose->position->z = 0.0;

        double wx = message.angular_velocity().x();
        double wy = message.angular_velocity().y();
        double wz = message.angular_velocity().z();
        double length = sqrt(wx * wx + wy * wy + wz * wz);

        if (length > 0.001) {
            angvel_arrow.shaft_length = length * 0.1;
            angvel_arrow.head_length = angvel_arrow.shaft_length * 0.2;
            angvel_arrow.shaft_diameter = 0.02;
            angvel_arrow.head_diameter = 0.05;

            angvel_arrow.pose->orientation = foxglove::schemas::Quaternion();
            angvel_arrow.pose->orientation->x = 0.0;
            angvel_arrow.pose->orientation->y = 0.0;
            angvel_arrow.pose->orientation->z = 0.0;
            angvel_arrow.pose->orientation->w = 1.0;

            angvel_arrow.color = CreateColor(0.0, 0.0, 1.0, 1.0);
            entity.arrows.push_back(angvel_arrow);
        }
    }

    if (!entity.arrows.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted Imu to SceneUpdate with " << entity.arrows.size()
          << " arrows";

    return scene_update;
}

foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::sensor_msgs::Range& message) {
    foxglove::schemas::SceneUpdate scene_update;

    // 创建场景实体
    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    entity.id = "range_sensor";
    entity.frame_locked = false;

    // 创建圆锥体显示传感器范围
    if (message.range() >= message.min_range() &&
        message.range() <= message.max_range()) {
        foxglove::schemas::CylinderPrimitive cone;
        cone.pose = foxglove::schemas::Pose();
        cone.pose->position = foxglove::schemas::Vector3();
        cone.pose->position->x = message.range() / 2.0;
        cone.pose->position->y = 0.0;
        cone.pose->position->z = 0.0;

        cone.pose->orientation = foxglove::schemas::Quaternion();
        cone.pose->orientation->x = 0.0;
        cone.pose->orientation->y = 0.0;
        cone.pose->orientation->z = 0.0;
        cone.pose->orientation->w = 1.0;

        // 设置大小
        cone.size = foxglove::schemas::Vector3();
        cone.size->x = message.range();
        double radius = message.range() * tan(message.field_of_view() / 2.0);
        cone.size->y = radius * 2.0;
        cone.size->z = radius * 2.0;

        // 设置圆锥形状
        cone.top_scale = 0.1;
        cone.bottom_scale = 1.0;

        cone.color = CreateColor(1.0, 1.0, 0.0, 0.3);
        entity.cylinders.push_back(cone);
    }

    if (!entity.cylinders.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted Range sensor to SceneUpdate";

    return scene_update;
}

foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::sensor_msgs::ChannelFloat32& message) {
    // ChannelFloat32 是 PointCloud 的辅助数据，不需要单独可视化
    (void)message;
    return foxglove::schemas::SceneUpdate();
}

foxglove::schemas::RawImage ToFoxgloveImpl(
    const autonomy::commsgs::proto::sensor_msgs::CompressedImage& message) {
    foxglove::schemas::RawImage raw_image;

    SetRawImageHeader(raw_image, ExtractTimestamp(message),
                      ExtractFrameId(message));

    raw_image.width = 0;
    raw_image.height = 0;
    raw_image.encoding = message.format();
    raw_image.step = 0;

    // 复制压缩图像数据
    if (message.data().size() > 0) {
        raw_image.data.resize(message.data().size());
        std::memcpy(raw_image.data.data(), message.data().data(),
                    message.data().size());
    }

    AINFO << "Converted CompressedImage to RawImage: "
          << "format=" << raw_image.encoding
          << ", data_size=" << raw_image.data.size() << " bytes";

    return raw_image;
}

foxglove::schemas::RawImage ToFoxgloveImpl(
    const autonomy::commsgs::proto::sensor_msgs::Image& message) {
    foxglove::schemas::RawImage raw_image;

    SetRawImageHeader(raw_image, ExtractTimestamp(message),
                      ExtractFrameId(message));

    raw_image.width = message.width();
    raw_image.height = message.height();
    raw_image.encoding = message.encoding();
    raw_image.step = message.step();

    // 复制图像数据
    if (message.data().size() > 0) {
        raw_image.data.resize(message.data().size());
        std::memcpy(raw_image.data.data(), message.data().data(),
                    message.data().size());
    }

    AINFO << "Converted Image to RawImage: " << raw_image.width << "x"
          << raw_image.height << ", encoding=" << raw_image.encoding
          << ", data_size=" << raw_image.data.size() << " bytes";

    return raw_image;
}

foxglove::schemas::RawImage ToFoxgloveImpl(const cv::Mat& image) {
    foxglove::schemas::RawImage raw_image;

    // 检查图像是否为空
    if (image.empty()) {
        AWARN << "Input cv::Mat is empty, returning empty RawImage";
        return raw_image;
    }

    // 设置头部信息（cv::Mat 没有时间戳和 frame_id，使用 nullopt）
    SetRawImageHeader(raw_image, std::nullopt, std::nullopt);

    // 设置图像尺寸
    raw_image.width = static_cast<uint32_t>(image.cols);
    raw_image.height = static_cast<uint32_t>(image.rows);

    // 设置步长（每行的字节数）
    raw_image.step = static_cast<uint32_t>(image.step);

    // 根据 cv::Mat 类型设置编码格式
    switch (image.type()) {
        case CV_8UC1:
            raw_image.encoding = "mono8";
            break;
        case CV_8UC3:
            raw_image.encoding = "bgr8";  // OpenCV 默认使用 BGR 顺序
            break;
        case CV_8UC4:
            raw_image.encoding = "bgra8";
            break;
        case CV_16UC1:
            raw_image.encoding = "mono16";
            break;
        case CV_16SC1:
            raw_image.encoding = "16SC1";
            break;
        case CV_32SC1:
            raw_image.encoding = "32SC1";
            break;
        case CV_32FC1:
            raw_image.encoding = "32FC1";
            break;
        case CV_32FC3:
            raw_image.encoding = "32FC3";
            break;
        case CV_64FC1:
            raw_image.encoding = "64FC1";
            break;
        default:
            AWARN << "Unknown cv::Mat type: " << image.type()
                  << ", defaulting to unknown encoding";
            raw_image.encoding = "unknown";
            break;
    }

    // 复制图像数据
    size_t data_size = image.total() * image.elemSize();
    if (data_size > 0) {
        raw_image.data.resize(data_size);
        std::memcpy(raw_image.data.data(), image.data, data_size);
    }

    AINFO << "Converted cv::Mat to RawImage: " << raw_image.width << "x"
          << raw_image.height << ", encoding=" << raw_image.encoding
          << ", type=" << image.type() << ", data_size=" << data_size
          << " bytes";

    return raw_image;
}

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy

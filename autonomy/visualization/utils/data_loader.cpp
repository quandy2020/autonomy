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

#include "autonomy/visualization/utils/data_loader.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/proto/builtin_interfaces.pb.h"
#include "autonomy/commsgs/proto/std_msgs.pb.h"

namespace autonomy {
namespace visualization {
namespace utils {

// ========== 图片加载功能 ==========

bool DataLoader::LoadImageFromFile(const std::string& file_path,
                                   commsgs::proto::sensor_msgs::Image& image,
                                   const std::string& frame_id) {
    // 检查文件是否存在
    if (!std::filesystem::exists(file_path)) {
        AERROR << "Image file does not exist: " << file_path;
        return false;
    }

    // 使用 OpenCV 加载图片
    cv::Mat cv_image = cv::imread(file_path, cv::IMREAD_COLOR);
    if (cv_image.empty()) {
        AERROR << "Failed to load image from: " << file_path;
        return false;
    }

    // 转换为 commsgs Image
    return ConvertCvMatToImage(cv_image, image, frame_id);
}

bool DataLoader::LoadImagesFromDirectory(
    const std::string& directory_path,
    std::vector<commsgs::proto::sensor_msgs::Image>& images,
    const std::string& frame_id) {
    // 检查目录是否存在
    if (!std::filesystem::exists(directory_path)) {
        AERROR << "Directory does not exist: " << directory_path;
        return false;
    }

    if (!std::filesystem::is_directory(directory_path)) {
        AERROR << "Path is not a directory: " << directory_path;
        return false;
    }

    // 支持的图片格式扩展名
    std::vector<std::string> supported_extensions = {".jpg", ".jpeg", ".png",
                                                     ".bmp", ".tiff", ".tif"};

    // 遍历目录中的所有文件
    for (const auto& entry :
         std::filesystem::directory_iterator(directory_path)) {
        if (std::filesystem::is_regular_file(entry)) {
            std::string extension = entry.path().extension().string();
            std::transform(extension.begin(), extension.end(),
                           extension.begin(), ::tolower);

            if (std::find(supported_extensions.begin(),
                          supported_extensions.end(),
                          extension) != supported_extensions.end()) {
                commsgs::proto::sensor_msgs::Image image;
                if (LoadImageFromFile(entry.path().string(), image, frame_id)) {
                    images.push_back(image);
                } else {
                    AWARN << "Failed to load image: " << entry.path().string();
                }
            }
        }
    }

    AINFO << "Loaded " << images.size() << " images from " << directory_path;
    return !images.empty();
}

bool DataLoader::ConvertCvMatToImage(const cv::Mat& cv_image,
                                     commsgs::proto::sensor_msgs::Image& image,
                                     const std::string& frame_id) {
    if (cv_image.empty()) {
        AERROR << "Input cv::Mat is empty";
        return false;
    }

    // 设置 header
    auto* header = image.mutable_header();
    header->set_frame_id(frame_id);
    SetTimestamp(*header->mutable_stamp());

    // 设置图像尺寸
    image.set_width(static_cast<uint32_t>(cv_image.cols));
    image.set_height(static_cast<uint32_t>(cv_image.rows));

    // 设置编码格式和步长
    int channels = cv_image.channels();
    int step = cv_image.cols * channels;
    image.set_step(static_cast<uint32_t>(step));
    image.set_is_bigendian(false);

    // 根据通道数设置编码
    switch (channels) {
        case 1:
            image.set_encoding("mono8");
            break;
        case 3:
            // OpenCV 默认使用 BGR 顺序
            image.set_encoding("bgr8");
            break;
        case 4:
            image.set_encoding("bgra8");
            break;
        default:
            AERROR << "Unsupported number of channels: " << channels;
            return false;
    }

    // 复制图像数据
    size_t data_size = cv_image.total() * cv_image.elemSize();
    image.mutable_data()->resize(data_size);
    std::memcpy(image.mutable_data()->data(), cv_image.data, data_size);

    return true;
}

// ========== 路径生成功能 ==========

commsgs::proto::planning_msgs::Path DataLoader::GenerateCircularPath(
    double center_x, double center_y, double radius, int num_points,
    const std::string& frame_id) {
    commsgs::proto::planning_msgs::Path path;

    // 设置 header
    auto* header = path.mutable_header();
    header->set_frame_id(frame_id);
    SetTimestamp(*header->mutable_stamp());

    // 生成圆形路径点
    for (int i = 0; i < num_points; ++i) {
        double angle = 2.0 * M_PI * i / num_points;
        double x = center_x + radius * std::cos(angle);
        double y = center_y + radius * std::sin(angle);

        auto* pose_stamped = path.add_poses();
        pose_stamped->mutable_header()->set_frame_id(frame_id);
        pose_stamped->mutable_header()->mutable_stamp()->CopyFrom(
            *header->mutable_stamp());

        auto* pose = pose_stamped->mutable_pose();
        pose->mutable_position()->set_x(x);
        pose->mutable_position()->set_y(y);
        pose->mutable_position()->set_z(0.0);

        // 设置方向（朝向路径方向）
        SetPoseOrientation(*pose, angle + M_PI / 2.0);
    }

    return path;
}

commsgs::proto::planning_msgs::Path DataLoader::GenerateStraightPath(
    double start_x, double start_y, double end_x, double end_y, int num_points,
    const std::string& frame_id) {
    commsgs::proto::planning_msgs::Path path;

    // 设置 header
    auto* header = path.mutable_header();
    header->set_frame_id(frame_id);
    SetTimestamp(*header->mutable_stamp());

    // 计算方向角度
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double angle = std::atan2(dy, dx);

    // 生成直线路径点
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        double x = start_x + t * dx;
        double y = start_y + t * dy;

        auto* pose_stamped = path.add_poses();
        pose_stamped->mutable_header()->set_frame_id(frame_id);
        pose_stamped->mutable_header()->mutable_stamp()->CopyFrom(
            *header->mutable_stamp());

        auto* pose = pose_stamped->mutable_pose();
        pose->mutable_position()->set_x(x);
        pose->mutable_position()->set_y(y);
        pose->mutable_position()->set_z(0.0);

        // 设置方向
        SetPoseOrientation(*pose, angle);
    }

    return path;
}

commsgs::proto::planning_msgs::Path DataLoader::GenerateRectangularPath(
    double center_x, double center_y, double width, double height,
    int num_points_per_side, const std::string& frame_id) {
    commsgs::proto::planning_msgs::Path path;

    // 设置 header
    auto* header = path.mutable_header();
    header->set_frame_id(frame_id);
    SetTimestamp(*header->mutable_stamp());

    // 计算矩形四个角点
    double half_width = width / 2.0;
    double half_height = height / 2.0;

    std::vector<std::pair<double, double>> corners = {
        {center_x - half_width, center_y - half_height},  // 左下
        {center_x + half_width, center_y - half_height},  // 右下
        {center_x + half_width, center_y + half_height},  // 右上
        {center_x - half_width, center_y + half_height}   // 左上
    };

    // 生成四条边的路径点
    for (size_t i = 0; i < corners.size(); ++i) {
        size_t next_i = (i + 1) % corners.size();
        double start_x = corners[i].first;
        double start_y = corners[i].second;
        double end_x = corners[next_i].first;
        double end_y = corners[next_i].second;

        for (int j = 0; j < num_points_per_side; ++j) {
            double t = static_cast<double>(j) / num_points_per_side;
            double x = start_x + t * (end_x - start_x);
            double y = start_y + t * (end_y - start_y);

            auto* pose_stamped = path.add_poses();
            pose_stamped->mutable_header()->set_frame_id(frame_id);
            pose_stamped->mutable_header()->mutable_stamp()->CopyFrom(
                *header->mutable_stamp());

            auto* pose = pose_stamped->mutable_pose();
            pose->mutable_position()->set_x(x);
            pose->mutable_position()->set_y(y);
            pose->mutable_position()->set_z(0.0);

            // 计算方向
            double dx = end_x - start_x;
            double dy = end_y - start_y;
            double angle = std::atan2(dy, dx);
            SetPoseOrientation(*pose, angle);
        }
    }

    return path;
}

commsgs::proto::planning_msgs::Path DataLoader::GeneratePathFromPoints(
    const std::vector<std::pair<double, double>>& points,
    const std::string& frame_id) {
    commsgs::proto::planning_msgs::Path path;

    if (points.empty()) {
        AWARN << "Empty points list, returning empty path";
        return path;
    }

    // 设置 header
    auto* header = path.mutable_header();
    header->set_frame_id(frame_id);
    SetTimestamp(*header->mutable_stamp());

    // 生成路径点
    for (size_t i = 0; i < points.size(); ++i) {
        double x = points[i].first;
        double y = points[i].second;

        auto* pose_stamped = path.add_poses();
        pose_stamped->mutable_header()->set_frame_id(frame_id);
        pose_stamped->mutable_header()->mutable_stamp()->CopyFrom(
            *header->mutable_stamp());

        auto* pose = pose_stamped->mutable_pose();
        pose->mutable_position()->set_x(x);
        pose->mutable_position()->set_y(y);
        pose->mutable_position()->set_z(0.0);

        // 计算方向（朝向下一个点）
        double angle = 0.0;
        if (i + 1 < points.size()) {
            double dx = points[i + 1].first - x;
            double dy = points[i + 1].second - y;
            angle = std::atan2(dy, dx);
        } else if (i > 0) {
            // 最后一个点，使用前一个点的方向
            double dx = x - points[i - 1].first;
            double dy = y - points[i - 1].second;
            angle = std::atan2(dy, dx);
        }
        SetPoseOrientation(*pose, angle);
    }

    return path;
}

commsgs::proto::planning_msgs::Path DataLoader::GenerateSpiralPath(
    double center_x, double center_y, double start_radius, double end_radius,
    int num_turns, int num_points, const std::string& frame_id) {
    commsgs::proto::planning_msgs::Path path;

    // 设置 header
    auto* header = path.mutable_header();
    header->set_frame_id(frame_id);
    SetTimestamp(*header->mutable_stamp());

    // 生成螺旋路径点
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        double angle = 2.0 * M_PI * num_turns * t;
        double radius = start_radius + (end_radius - start_radius) * t;

        double x = center_x + radius * std::cos(angle);
        double y = center_y + radius * std::sin(angle);

        auto* pose_stamped = path.add_poses();
        pose_stamped->mutable_header()->set_frame_id(frame_id);
        pose_stamped->mutable_header()->mutable_stamp()->CopyFrom(
            *header->mutable_stamp());

        auto* pose = pose_stamped->mutable_pose();
        pose->mutable_position()->set_x(x);
        pose->mutable_position()->set_y(y);
        pose->mutable_position()->set_z(0.0);

        // 设置方向（朝向螺旋方向）
        double tangent_angle = angle + M_PI / 2.0;
        SetPoseOrientation(*pose, tangent_angle);
    }

    return path;
}

// ========== 辅助函数 ==========

void DataLoader::SetTimestamp(commsgs::proto::builtin_interfaces::Time& stamp) {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        duration - seconds);
    stamp.set_sec(seconds.count());
    stamp.set_nanosec(nanoseconds.count());
}

void DataLoader::SetPoseOrientation(commsgs::proto::geometry_msgs::Pose& pose,
                                    double angle) {
    // 将角度转换为四元数
    pose.mutable_orientation()->set_x(0.0);
    pose.mutable_orientation()->set_y(0.0);
    pose.mutable_orientation()->set_z(std::sin(angle / 2.0));
    pose.mutable_orientation()->set_w(std::cos(angle / 2.0));
}

}  // namespace utils
}  // namespace visualization
}  // namespace autonomy

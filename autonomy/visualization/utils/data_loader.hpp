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

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "autonomy/commsgs/proto/planning_msgs.pb.h"
#include "autonomy/commsgs/proto/sensor_msgs.pb.h"

namespace autonomy {
namespace visualization {
namespace utils {

/**
 * @brief DataLoader：辅助测试工具类
 *
 * 提供以下功能：
 * 1. 从文件或目录加载图片并转换为 commsgs 格式
 * 2. 生成测试路径数据
 * 3. 其他测试辅助功能
 */
class DataLoader
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(DataLoader)

    /**
     * @brief 从文件加载单张图片
     *
     * @param file_path 图片文件路径
     * @param image 输出的 commsgs Image 消息
     * @param frame_id 图像 frame_id（默认为 "camera_frame"）
     * @return true 如果成功，false 否则
     */
    static bool LoadImageFromFile(const std::string& file_path,
                                  commsgs::proto::sensor_msgs::Image& image,
                                  const std::string& frame_id = "camera_frame");

    /**
     * @brief 从目录加载多张图片
     *
     * @param directory_path 图片目录路径
     * @param images 输出的 commsgs Image 消息列表
     * @param frame_id 图像 frame_id（默认为 "camera_frame"）
     * @return true 如果成功，false 否则
     */
    static bool LoadImagesFromDirectory(
        const std::string& directory_path,
        std::vector<commsgs::proto::sensor_msgs::Image>& images,
        const std::string& frame_id = "camera_frame");

    /**
     * @brief 从 OpenCV Mat 转换为 commsgs Image
     *
     * @param cv_image OpenCV Mat 图像
     * @param image 输出的 commsgs Image 消息
     * @param frame_id 图像 frame_id（默认为 "camera_frame"）
     * @return true 如果成功，false 否则
     */
    static bool ConvertCvMatToImage(
        const cv::Mat& cv_image, commsgs::proto::sensor_msgs::Image& image,
        const std::string& frame_id = "camera_frame");

    /**
     * @brief 生成圆形路径
     *
     * @param center_x 圆心 x 坐标
     * @param center_y 圆心 y 坐标
     * @param radius 半径
     * @param num_points 路径点数量
     * @param frame_id 路径 frame_id（默认为 "map"）
     * @return 生成的 Path 消息
     */
    static commsgs::proto::planning_msgs::Path GenerateCircularPath(
        double center_x, double center_y, double radius, int num_points = 20,
        const std::string& frame_id = "map");

    /**
     * @brief 生成直线路径
     *
     * @param start_x 起点 x 坐标
     * @param start_y 起点 y 坐标
     * @param end_x 终点 x 坐标
     * @param end_y 终点 y 坐标
     * @param num_points 路径点数量
     * @param frame_id 路径 frame_id（默认为 "map"）
     * @return 生成的 Path 消息
     */
    static commsgs::proto::planning_msgs::Path GenerateStraightPath(
        double start_x, double start_y, double end_x, double end_y,
        int num_points = 10, const std::string& frame_id = "map");

    /**
     * @brief 生成矩形路径
     *
     * @param center_x 中心 x 坐标
     * @param center_y 中心 y 坐标
     * @param width 宽度
     * @param height 高度
     * @param num_points_per_side 每边的点数
     * @param frame_id 路径 frame_id（默认为 "map"）
     * @return 生成的 Path 消息
     */
    static commsgs::proto::planning_msgs::Path GenerateRectangularPath(
        double center_x, double center_y, double width, double height,
        int num_points_per_side = 5, const std::string& frame_id = "map");

    /**
     * @brief 从点列表生成路径
     *
     * @param points 点列表 (x, y) 对
     * @param frame_id 路径 frame_id（默认为 "map"）
     * @return 生成的 Path 消息
     */
    static commsgs::proto::planning_msgs::Path GeneratePathFromPoints(
        const std::vector<std::pair<double, double>>& points,
        const std::string& frame_id = "map");

    /**
     * @brief 生成螺旋路径
     *
     * @param center_x 中心 x 坐标
     * @param center_y 中心 y 坐标
     * @param start_radius 起始半径
     * @param end_radius 结束半径
     * @param num_turns 圈数
     * @param num_points 路径点数量
     * @param frame_id 路径 frame_id（默认为 "map"）
     * @return 生成的 Path 消息
     */
    static commsgs::proto::planning_msgs::Path GenerateSpiralPath(
        double center_x, double center_y, double start_radius,
        double end_radius, int num_turns, int num_points = 50,
        const std::string& frame_id = "map");

private:
    /**
     * @brief 设置消息的时间戳
     *
     * @param stamp 时间戳消息
     */
    static void SetTimestamp(commsgs::proto::builtin_interfaces::Time& stamp);

    /**
     * @brief 设置路径中位姿的方向（朝向路径方向）
     *
     * @param pose 位姿消息
     * @param angle 角度（弧度）
     */
    static void SetPoseOrientation(commsgs::proto::geometry_msgs::Pose& pose,
                                   double angle);
};

}  // namespace utils
}  // namespace visualization
}  // namespace autonomy

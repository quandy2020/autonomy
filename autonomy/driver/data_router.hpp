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

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/driver/driver_engine.hpp"
#include "autonomy/driver/proto/driver_options.pb.h"
#include "autonomy/sensor/data.hpp"

namespace autonomy {
namespace driver {

/**
 * @class DataRouter
 * @brief 数据路由器，用于在 ROS2 传感器数据和内部驱动数据之间进行路由
 *
 * DataRouter 负责：
 * 1. 检测数据源类型（ROS2 或内部驱动）
 * 2. 如果使用 ROS2 传感器数据，则转发到内部系统
 * 3. 如果使用内部驱动数据，则读取驱动数据并发送到目标
 */
class DataRouter
{
public:
    /**
     * Define DataRouter::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(DataRouter)

    /**
     * @brief 数据源类型枚举
     */
    enum class DataSource {
        ROS2,   // ROS2 传感器数据
        DRIVER  // 内部驱动数据
    };

    /**
     * @brief 构造函数
     * @param node Autolink 节点指针（生命周期由外部管理）
     * @param driver_engine 驱动引擎指针（生命周期由外部管理）
     */
    explicit DataRouter(::autolink::Node* node, DriverEngine* driver_engine);

    /**
     * @brief 析构函数
     */
    ~DataRouter();

    DataRouter(const DataRouter&) = delete;
    DataRouter& operator=(const DataRouter&) = delete;

    /**
     * @brief 初始化数据路由器
     * @param options 驱动配置选项
     * @return true 成功，false 失败
     */
    bool Initialize(const proto::DriverOptions& options);

    /**
     * @brief 启动数据路由器
     */
    void Start();

    /**
     * @brief 停止数据路由器
     */
    void Stop();

    /**
     * @brief 设置数据源类型
     * @param sensor_id 传感器ID
     * @param source 数据源类型
     */
    void SetDataSource(const std::string& sensor_id, DataSource source);

    /**
     * @brief 获取数据源类型
     * @param sensor_id 传感器ID
     * @return 数据源类型
     */
    DataSource GetDataSource(const std::string& sensor_id) const;

    /**
     * @brief 从 ROS2 接收传感器数据并转发
     * @param sensor_id 传感器ID
     * @param data 传感器数据
     */
    void ForwardFromRos2(const std::string& sensor_id, const std::shared_ptr<sensor::Data>& data);

    /**
     * @brief 从内部驱动读取数据并发送
     * @param sensor_id 传感器ID
     */
    void ReadAndSendFromDriver(const std::string& sensor_id);

    /**
     * @brief 注册数据接收处理器（用于接收转发后的数据）
     * @param target 目标名称（如 "map", "localization" 等）
     * @param handler 数据处理器函数
     */
    void RegisterTargetHandler(const std::string& target,
                               std::function<void(const std::string&, const std::shared_ptr<sensor::Data>&)> handler);

    /**
     * @brief 取消注册数据接收处理器
     * @param target 目标名称
     */
    void UnregisterTargetHandler(const std::string& target);

    /**
     * @brief 转发数据到指定目标
     * @param sensor_id 传感器ID
     * @param data 传感器数据
     * @param targets 目标列表（如果为空，则使用默认目标）
     */
    void ForwardToTargets(const std::string& sensor_id, const std::shared_ptr<sensor::Data>& data,
                          const std::vector<std::string>& targets = {});

    /**
     * @brief 发布激光扫描数据
     * @param topic 发布主题
     * @param message 激光扫描消息
     * @return true 成功，false 失败
     */
    bool PublishLaserScan(const std::string& topic, const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& message);

    /**
     * @brief 发布点云数据（PointCloud2）
     * @param topic 发布主题
     * @param message 点云消息
     * @return true 成功，false 失败
     */
    bool PublishPointCloud2(const std::string& topic,
                            const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& message);

    /**
     * @brief 发布点云数据（PointCloud）
     * @param topic 发布主题
     * @param message 点云消息
     * @return true 成功，false 失败
     */
    bool PublishPointCloud(const std::string& topic, const std::shared_ptr<commsgs::sensor_msgs::PointCloud>& message);

    /**
     * @brief 发布IMU数据
     * @param topic 发布主题
     * @param message IMU消息
     * @return true 成功，false 失败
     */
    bool PublishImu(const std::string& topic, const std::shared_ptr<commsgs::sensor_msgs::Imu>& message);

    /**
     * @brief 发布里程计数据
     * @param topic 发布主题
     * @param message 里程计消息
     * @return true 成功，false 失败
     */
    bool PublishOdometry(const std::string& topic, const std::shared_ptr<commsgs::planning_msgs::Odometry>& message);

    /**
     * @brief 发布图像数据
     * @param topic 发布主题
     * @param message 图像消息
     * @return true 成功，false 失败
     */
    bool PublishImage(const std::string& topic, const std::shared_ptr<commsgs::sensor_msgs::Image>& message);

    /**
     * @brief 发布测距传感器数据
     * @param topic 发布主题
     * @param message 测距消息
     * @return true 成功，false 失败
     */
    bool PublishRange(const std::string& topic, const std::shared_ptr<commsgs::sensor_msgs::Range>& message);

private:
    /**
     * @brief 检测数据源类型（根据配置和实际可用性）
     * @param sensor_id 传感器ID
     * @return 数据源类型
     */
    DataSource DetectDataSource(const std::string& sensor_id) const;

    /**
     * @brief 传感器发布器信息
     */
    struct SensorPublisher {
        std::string topic;
        std::string sensor_type;
        std::shared_ptr<void> writer;  // Autolink Writer指针（类型擦除）
    };

    // Autolink节点（生命周期由外部管理）
    ::autolink::Node* node_;

    // 驱动引擎（生命周期由外部管理）
    DriverEngine* driver_engine_;

    // 数据源类型映射表（传感器ID -> 数据源类型）
    std::map<std::string, DataSource> data_sources_;

    // 目标处理器映射表（目标名称 -> 处理器函数）
    std::map<std::string, std::function<void(const std::string&, const std::shared_ptr<sensor::Data>&)>>
        target_handlers_;

    // 默认转发目标列表
    std::vector<std::string> default_forward_targets_;

    // 传感器发布器映射表（topic -> SensorPublisher）
    std::map<std::string, SensorPublisher> sensor_publishers_;

    // 互斥锁
    mutable std::mutex mutex_;

    // 状态标志
    bool initialized_{false};
    bool started_{false};
};

}  // namespace driver
}  // namespace autonomy

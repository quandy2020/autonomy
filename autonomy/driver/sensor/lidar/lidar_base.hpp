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
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/driver/common/driver_interface.hpp"
#include "autonomy/driver/proto/driver_options.pb.h"
#include "autonomy/driver/proto/lidar_options.pb.h"
#include "autonomy/sensor/data.hpp"

namespace autonomy {
namespace driver {
namespace sensor {
namespace lidar {

/**
 * @class LidarBase
 * @brief 激光雷达驱动基类，继承 DriverInterface，提供激光雷达驱动的通用功能
 *
 * LidarBase 提供：
 * 1. 激光雷达数据读取和转换的统一接口
 * 2. 配置管理和生命周期管理
 * 3. 数据回调机制
 * 4. 支持不同硬件型号的插件化实现
 *
 * 子类需要实现：
 * - ReadLaserScanData() 或 ReadPointCloudData() - 从硬件读取激光雷达数据
 * - GetHardwareModel() - 返回硬件型号名称
 */
class LidarBase : public common::DriverInterface
{
public:
    /**
     * Define LidarBase::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(LidarBase)

    /**
     * @brief 构造函数
     */
    LidarBase();

    /**
     * @brief 析构函数
     */
    virtual ~LidarBase();

    LidarBase(const LidarBase&) = delete;
    LidarBase& operator=(const LidarBase&) = delete;

    /**
     * @brief 配置驱动器
     * @param name 驱动器名称
     * @param options 驱动器配置选项
     * @return true 成功，false 失败
     */
    bool Configure(const std::string& name,
                   const proto::DriverOptions& options) override;

    /**
     * @brief 初始化驱动器
     * @return true 成功，false 失败
     */
    bool Initialize() override;

    /**
     * @brief 启动驱动器
     */
    void Start() override;

    /**
     * @brief 停止驱动器
     */
    void Stop() override;

    /**
     * @brief 清理资源
     */
    void Cleanup() override;

    /**
     * @brief 获取驱动器名称
     * @return 驱动器名称
     */
    std::string GetName() const override;

    /**
     * @brief 获取传感器ID列表
     * @return 传感器ID列表
     */
    std::vector<std::string> GetSensorIds() const override;

    /**
     * @brief 检查传感器是否已注册
     * @param sensor_id 传感器ID
     * @return true 已注册，false 未注册
     */
    bool IsSensorRegistered(const std::string& sensor_id) const override;

    /**
     * @brief 注册传感器数据处理器
     * @param sensor_id 传感器ID
     * @param handler 数据处理器函数
     * @return true 成功，false 失败
     */
    bool RegisterSensorHandler(
        const std::string& sensor_id,
        std::function<void(const std::string&,
                           const std::shared_ptr<autonomy::sensor::Data>&)>
            handler) override;

    /**
     * @brief 取消注册传感器数据处理器
     * @param sensor_id 传感器ID
     */
    void UnregisterSensorHandler(const std::string& sensor_id) override;

    /**
     * @brief 获取硬件型号名称（子类实现）
     * @return 硬件型号名称，如 "RPLIDAR A1", "Velodyne VLP-16", "Livox Mid-40"
     * 等
     */
    virtual std::string GetHardwareModel() const = 0;

    /**
     * @brief 获取驱动器版本
     * @return 版本字符串
     */
    virtual std::string GetVersion() const {
        return "1.0.0";
    }

    /**
     * @brief 检查硬件连接状态
     * @return true 已连接，false 未连接
     */
    virtual bool IsConnected() const = 0;

protected:
    /**
     * @brief 从硬件读取 2D 激光扫描数据（子类实现，用于 LASER_SCAN 类型）
     * @param sensor_id 传感器ID
     * @return 激光扫描数据消息，如果读取失败返回 nullptr
     */
    virtual std::shared_ptr<commsgs::sensor_msgs::LaserScan> ReadLaserScanData(
        const std::string& sensor_id) {
        (void)sensor_id;
        return nullptr;
    }

    /**
     * @brief 从硬件读取 3D 点云数据（子类实现，用于 POINT_CLOUD2 类型）
     * @param sensor_id 传感器ID
     * @return 点云数据消息，如果读取失败返回 nullptr
     */
    virtual std::shared_ptr<commsgs::sensor_msgs::PointCloud2>
    ReadPointCloudData(const std::string& sensor_id) {
        (void)sensor_id;
        return nullptr;
    }

    /**
     * @brief 处理读取到的激光扫描数据并转发
     * @param sensor_id 传感器ID
     * @param scan_msg 激光扫描消息
     */
    void ProcessLaserScanData(
        const std::string& sensor_id,
        const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& scan_msg);

    /**
     * @brief 处理读取到的点云数据并转发
     * @param sensor_id 传感器ID
     * @param cloud_msg 点云消息
     */
    void ProcessPointCloudData(
        const std::string& sensor_id,
        const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& cloud_msg);

    /**
     * @brief 数据读取线程函数
     */
    void DataReadingThread();

    // 驱动器名称
    std::string name_;

    // 激光雷达配置选项映射表（sensor_id -> LidarOptions）
    std::map<std::string, proto::LidarOptions> lidar_configs_;

    // 传感器数据处理器映射表（sensor_id -> handler）
    std::map<std::string, std::function<void(
                              const std::string&,
                              const std::shared_ptr<autonomy::sensor::Data>&)>>
        handlers_;

    // 互斥锁
    mutable std::mutex mutex_;

    // 状态标志
    bool configured_{false};
    bool initialized_{false};
    bool started_{false};

    // 数据读取线程
    std::thread data_reading_thread_;
    std::atomic<bool> stop_thread_{false};
};

}  // namespace lidar
}  // namespace sensor
}  // namespace driver
}  // namespace autonomy

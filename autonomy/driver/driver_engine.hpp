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
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/driver/common/driver_interface.hpp"
#include "autonomy/driver/proto/driver_options.pb.h"
#include "autonomy/sensor/data.hpp"

namespace autonomy {
namespace driver {

/**
 * @class DriverEngine
 * @brief 驱动引擎类，统一管理传感器数据源（ROS2 和硬件驱动）
 *
 * DriverEngine 负责：
 * 1. 管理两种数据源模式：
 *    - ROS2 模式：通过 AddSensorData() 接收 ROS2 传感器数据
 *    - Driver 模式：通过硬件驱动（DriverInterface）读取传感器数据
 * 2. 支持从配置文件加载传感器配置（ROS2 订阅或硬件驱动）
 * 3. 提供统一的传感器数据接口，将数据转换为 sensor::Data 并转发
 * 4. 数据流向：
 *    ROS2 模式：AddSensorData() -> 转换为 sensor::Data -> ForwardSensorData()
 * -> DataRouter Driver 模式：硬件驱动读取 -> 转换为 sensor::Data ->
 * ForwardSensorData() -> DataRouter
 *
 * 设计说明：
 * - ROS2 模式：传感器数据来自 ROS2 主题，通过 AddSensorData() 手动添加
 * - Driver 模式：传感器数据来自硬件驱动，通过 DriverInterface 实现类自动读取
 * - 两种模式可以同时使用，通过配置决定每个传感器的数据源
 */
class DriverEngine {
 public:
  /**
   * Define DriverEngine::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(DriverEngine)

  /**
   * @brief 传感器数据处理器函数类型
   */
  using SensorDataHandler =
      std::function<void(const std::string& sensor_id, const std::shared_ptr<sensor::Data>& data)>;

  /**
   * @brief 构造函数
   * @param node Autolink 节点指针（生命周期由外部管理）
   */
  explicit DriverEngine(::autolink::Node* node);

  /**
   * @brief 析构函数
   */
  ~DriverEngine();

  DriverEngine(const DriverEngine&) = delete;
  DriverEngine& operator=(const DriverEngine&) = delete;

  /**
   * @brief 初始化驱动引擎
   * @param options 驱动配置选项
   * @return true 成功，false 失败
   */
  bool Initialize(const proto::DriverOptions& options);

  /**
   * @brief 启动驱动引擎
   */
  void Start();

  /**
   * @brief 停止驱动引擎
   */
  void Stop();

  /**
   * @brief 注册传感器数据处理器
   * @param sensor_id 传感器ID
   * @param handler 数据处理器函数
   * @return true 成功，false 失败
   */
  bool RegisterSensorHandler(const std::string& sensor_id, SensorDataHandler handler);

  /**
   * @brief 取消注册传感器数据处理器
   * @param sensor_id 传感器ID
   */
  void UnregisterSensorHandler(const std::string& sensor_id);

  /**
   * @brief 订阅传感器数据
   * @param sensor_id 传感器ID
   * @param topic 传感器数据主题
   * @param sensor_type 传感器类型
   * @return true 成功，false 失败
   */
  bool SubscribeSensor(const std::string& sensor_id, const std::string& topic, const std::string& sensor_type);

  /**
   * @brief 取消订阅传感器数据
   * @param sensor_id 传感器ID
   */
  void UnsubscribeSensor(const std::string& sensor_id);

  /**
   * @brief 取消订阅所有传感器
   */
  void UnsubscribeAllSensors();

  /**
   * @brief 添加激光扫描数据
   * @param sensor_id 传感器ID
   * @param message 激光扫描消息
   */
  void AddSensorData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& message);

  /**
   * @brief 添加点云数据（PointCloud2）
   * @param sensor_id 传感器ID
   * @param message 点云消息
   */
  void AddSensorData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& message);

  /**
   * @brief 添加点云数据（PointCloud）
   * @param sensor_id 传感器ID
   * @param message 点云消息
   */
  void AddSensorData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::PointCloud>& message);

  /**
   * @brief 添加IMU数据
   * @param sensor_id 传感器ID
   * @param message IMU消息
   */
  void AddSensorData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::Imu>& message);

  /**
   * @brief 添加里程计数据
   * @param sensor_id 传感器ID
   * @param message 里程计消息
   */
  void AddSensorData(const std::string& sensor_id, const std::shared_ptr<commsgs::planning_msgs::Odometry>& message);

  /**
   * @brief 添加图像数据
   * @param sensor_id 传感器ID
   * @param message 图像消息
   */
  void AddSensorData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::Image>& message);

  /**
   * @brief 添加测距传感器数据
   * @param sensor_id 传感器ID
   * @param message 测距消息
   */
  void AddSensorData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::Range>& message);

  /**
   * @brief 添加GPS/导航卫星数据
   * @param sensor_id 传感器ID
   * @param message GPS消息（TODO: 需要定义 NavSatFix 消息类型）
   */
  void AddSensorData(const std::string& sensor_id, const std::shared_ptr<void>& message);

  /**
   * @brief 获取已注册的传感器ID列表
   * @return 传感器ID列表
   */
  std::vector<std::string> GetRegisteredSensors() const;

  /**
   * @brief 检查传感器是否已注册
   * @param sensor_id 传感器ID
   * @return true 已注册，false 未注册
   */
  bool IsSensorRegistered(const std::string& sensor_id) const;

  /**
   * @brief 注册硬件驱动（Driver 模式）
   * @param sensor_id 传感器ID
   * @param driver 硬件驱动接口（生命周期由外部管理）
   * @return true 成功，false 失败
   */
  bool RegisterHardwareDriver(const std::string& sensor_id, common::DriverInterface* driver);

  /**
   * @brief 取消注册硬件驱动
   * @param sensor_id 传感器ID
   */
  void UnregisterHardwareDriver(const std::string& sensor_id);

  /**
   * @brief 获取传感器数据源类型
   * @param sensor_id 传感器ID
   * @return true 使用硬件驱动，false 使用 ROS2
   */
  bool UsesHardwareDriver(const std::string& sensor_id) const;

  /**
   * @brief 获取所有已注册的硬件驱动传感器ID列表
   * @return 硬件驱动传感器ID列表
   */
  std::vector<std::string> GetHardwareDriverSensors() const;

 private:
  /**
   * @brief 传感器订阅信息（用于 ROS2 模式）
   */
  struct SensorSubscription {
    std::string sensor_id;
    std::string topic;
    std::string sensor_type;
    std::shared_ptr<void> reader;  // Autolink Reader指针（类型擦除）
  };

  /**
   * @brief 传感器数据源类型
   */
  enum class SensorDataSource {
    ROS2,   // ROS2 数据源（通过 AddSensorData 或订阅）
    DRIVER  // 硬件驱动数据源（通过 DriverInterface）
  };

  /**
   * @brief 转发传感器数据到所有注册的处理器
   * @param sensor_id 传感器ID
   * @param data 传感器数据
   */
  void ForwardSensorData(const std::string& sensor_id, const std::shared_ptr<sensor::Data>& data);

  /**
   * @brief 从硬件驱动读取数据并转发（内部方法，由驱动回调调用）
   * @param sensor_id 传感器ID
   * @param data 传感器数据
   */
  void OnHardwareDriverData(const std::string& sensor_id, const std::shared_ptr<sensor::Data>& data);

  // Autolink节点（生命周期由外部管理）
  ::autolink::Node* node_;

  // ROS2 模式：传感器订阅器映射表（sensor_id -> SensorSubscription）
  std::map<std::string, SensorSubscription> sensor_subscriptions_;

  // Driver 模式：硬件驱动映射表（sensor_id -> DriverInterface*）
  std::map<std::string, common::DriverInterface*> hardware_drivers_;

  // 传感器数据源类型映射表（sensor_id -> SensorDataSource）
  std::map<std::string, SensorDataSource> sensor_data_sources_;

  // 传感器数据处理器映射表（支持一个传感器多个处理器）
  std::multimap<std::string, SensorDataHandler> sensor_handlers_;

  // 互斥锁
  mutable std::mutex mutex_;

  // 状态标志
  bool initialized_{false};
  bool started_{false};
};

}  // namespace driver
}  // namespace autonomy

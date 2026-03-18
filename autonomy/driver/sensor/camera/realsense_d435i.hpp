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

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/driver/sensor/camera/camera_base.hpp"

namespace autonomy {
namespace driver {
namespace sensor {
namespace camera {

/**
 * @class RealSenseD435iDriver
 * @brief Intel RealSense D435i 相机驱动实现类
 *
 * RealSenseD435iDriver 提供：
 * 1. RealSense D435i 硬件初始化（USB 接口）
 * 2. 从硬件读取 RGB 图像和深度图像数据
 * 3. 数据格式转换和单位转换
 * 4. 硬件连接状态检测
 *
 * 硬件规格：
 * - RGB 相机：1920x1080 @ 30fps, 1280x720 @ 60fps
 * - 深度相机：1280x720 @ 30fps, 848x480 @ 60fps
 * - IMU：加速度计 + 陀螺仪（可选，由 IMU 驱动处理）
 * - USB 3.0 接口
 */
class RealSenseD435iDriver : public CameraBase {
 public:
  /**
   * Define RealSenseD435iDriver::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(RealSenseD435iDriver)

  /**
   * @brief 构造函数
   */
  RealSenseD435iDriver();

  /**
   * @brief 析构函数
   */
  ~RealSenseD435iDriver() override;

  RealSenseD435iDriver(const RealSenseD435iDriver&) = delete;
  RealSenseD435iDriver& operator=(const RealSenseD435iDriver&) = delete;

  /**
   * @brief 获取硬件型号名称
   * @return "RealSense D435i"
   */
  std::string GetHardwareModel() const override;

  /**
   * @brief 获取驱动器版本
   * @return 版本字符串
   */
  std::string GetVersion() const override;

  /**
   * @brief 检查硬件连接状态
   * @return true 已连接，false 未连接
   */
  bool IsConnected() const override;

  /**
   * @brief 初始化驱动器（重写基类方法，添加硬件初始化）
   * @return true 成功，false 失败
   */
  bool Initialize() override;

  /**
   * @brief 清理资源（重写基类方法，添加硬件清理）
   */
  void Cleanup() override;

 protected:
  /**
   * @brief 从硬件读取图像数据
   * @param sensor_id 传感器ID
   * @return 图像数据消息，如果读取失败返回 nullptr
   */
  std::shared_ptr<commsgs::sensor_msgs::Image> ReadImageData(const std::string& sensor_id) override;

  /**
   * @brief 从硬件读取深度图像数据
   * @param sensor_id 传感器ID
   * @return 深度图像数据消息，如果读取失败返回 nullptr
   */
  std::shared_ptr<commsgs::sensor_msgs::Image> ReadDepthData(const std::string& sensor_id) override;

 private:
  /**
   * @brief 初始化 RealSense D435i 硬件
   * @return true 成功，false 失败
   */
  bool InitializeHardware();

  // 硬件初始化标志
  bool hardware_initialized_;

  // 互斥锁（保护硬件访问）
  mutable std::mutex hardware_mutex_;

  // 传感器 frame_id 映射表（sensor_id -> frame_id）
  std::map<std::string, std::string> sensor_frame_ids_;

  // RealSense pipeline（前向声明，实际类型在 .cpp 中定义）
  // 使用 void* 避免在头文件中暴露 RealSense SDK 依赖
  void* pipeline_ptr_;  // 实际类型为 std::shared_ptr<rs2::pipeline> 或 nullptr
};

}  // namespace camera
}  // namespace sensor
}  // namespace driver
}  // namespace autonomy

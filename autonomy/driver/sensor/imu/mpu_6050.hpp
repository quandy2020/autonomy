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
#include "autonomy/driver/sensor/imu/imu_base.hpp"

namespace autonomy {
namespace driver {
namespace sensor {
namespace imu {

/**
 * @class Mpu6050Driver
 * @brief MPU6050 IMU 驱动实现类
 *
 * Mpu6050Driver 提供：
 * 1. MPU6050 硬件初始化（I2C 接口）
 * 2. 从硬件读取加速度计和陀螺仪数据
 * 3. 数据格式转换和单位转换
 * 4. 硬件连接状态检测
 *
 * 硬件规格：
 * - 3轴加速度计：±2g, ±4g, ±8g, ±16g
 * - 3轴陀螺仪：±250°/s, ±500°/s, ±1000°/s, ±2000°/s
 * - I2C 接口：默认地址 0x68
 * - 采样率：最高 1kHz
 */
class Mpu6050Driver : public ImuBase {
 public:
  /**
   * Define Mpu6050Driver::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(Mpu6050Driver)

  /**
   * @brief 构造函数
   */
  Mpu6050Driver();

  /**
   * @brief 析构函数
   */
  ~Mpu6050Driver() override;

  Mpu6050Driver(const Mpu6050Driver&) = delete;
  Mpu6050Driver& operator=(const Mpu6050Driver&) = delete;

  /**
   * @brief 获取硬件型号名称
   * @return "MPU6050"
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
   * @brief 从硬件读取 IMU 数据
   * @param sensor_id 传感器ID
   * @return IMU 数据消息，如果读取失败返回 nullptr
   */
  std::shared_ptr<commsgs::sensor_msgs::Imu> ReadImuData(const std::string& sensor_id) override;

 private:
  /**
   * @brief 初始化 MPU6050 硬件
   * @return true 成功，false 失败
   */
  bool InitializeHardware();

  /**
   * @brief 从 I2C 读取寄存器值
   * @param reg 寄存器地址
   * @return 寄存器值，失败返回 -1
   */
  int ReadRegister(uint8_t reg) const;

  /**
   * @brief 向 I2C 写入寄存器值
   * @param reg 寄存器地址
   * @param value 寄存器值
   * @return true 成功，false 失败
   */
  bool WriteRegister(uint8_t reg, uint8_t value);

  /**
   * @brief 读取加速度计原始数据
   * @param accel_x 加速度 X 轴（输出）
   * @param accel_y 加速度 Y 轴（输出）
   * @param accel_z 加速度 Z 轴（输出）
   * @return true 成功，false 失败
   */
  bool ReadAccelerometer(int16_t& accel_x, int16_t& accel_y, int16_t& accel_z);

  /**
   * @brief 读取陀螺仪原始数据
   * @param gyro_x 角速度 X 轴（输出）
   * @param gyro_y 角速度 Y 轴（输出）
   * @param gyro_z 角速度 Z 轴（输出）
   * @return true 成功，false 失败
   */
  bool ReadGyroscope(int16_t& gyro_x, int16_t& gyro_y, int16_t& gyro_z);

  /**
   * @brief 将原始加速度数据转换为 m/s²
   * @param raw_value 原始值
   * @param range 量程（2, 4, 8, 16）
   * @return 转换后的值（m/s²）
   */
  double ConvertAccelerometer(int16_t raw_value, int range);

  /**
   * @brief 将原始陀螺仪数据转换为 rad/s
   * @param raw_value 原始值
   * @param range 量程（250, 500, 1000, 2000）
   * @return 转换后的值（rad/s）
   */
  double ConvertGyroscope(int16_t raw_value, int range);

  // I2C 设备路径（如 "/dev/i2c-1"）
  std::string i2c_device_path_;

  // I2C 设备地址（默认 0x68）
  uint8_t i2c_address_;

  // I2C 文件描述符（-1 表示未打开）
  int i2c_fd_;

  // 加速度计量程（2, 4, 8, 16）
  int accel_range_;

  // 陀螺仪量程（250, 500, 1000, 2000）
  int gyro_range_;

  // 硬件初始化标志
  bool hardware_initialized_;

  // 互斥锁（保护硬件访问）
  mutable std::mutex hardware_mutex_;

  // 传感器 frame_id 映射表（sensor_id -> frame_id）
  std::map<std::string, std::string> sensor_frame_ids_;
};

}  // namespace imu
}  // namespace sensor
}  // namespace driver
}  // namespace autonomy

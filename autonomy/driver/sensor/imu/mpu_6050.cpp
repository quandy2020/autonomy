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

#include "autonomy/driver/sensor/imu/mpu_6050.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>

#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace driver {
namespace sensor {
namespace imu {

// MPU6050 寄存器地址定义
constexpr uint8_t MPU6050_ADDR = 0x68;
constexpr uint8_t MPU6050_WHO_AM_I = 0x75;
constexpr uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_SMPLRT_DIV = 0x19;
constexpr uint8_t MPU6050_CONFIG = 0x1A;
constexpr uint8_t MPU6050_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU6050_GYRO_XOUT_H = 0x43;
constexpr uint8_t MPU6050_TEMP_OUT_H = 0x41;

// MPU6050 寄存器值定义
constexpr uint8_t MPU6050_WHO_AM_I_VALUE = 0x68;
constexpr uint8_t MPU6050_PWR_MGMT_1_RESET = 0x80;
constexpr uint8_t MPU6050_PWR_MGMT_1_WAKEUP = 0x00;

Mpu6050Driver::Mpu6050Driver()
    : i2c_device_path_("/dev/i2c-1"),
      i2c_address_(MPU6050_ADDR),
      i2c_fd_(-1),
      accel_range_(16),   // ±16g
      gyro_range_(2000),  // ±2000°/s
      hardware_initialized_(false) {}

Mpu6050Driver::~Mpu6050Driver() {
    Cleanup();
}

std::string Mpu6050Driver::GetHardwareModel() const {
    return "MPU6050";
}

std::string Mpu6050Driver::GetVersion() const {
    return "1.0.0";
}

bool Mpu6050Driver::IsConnected() const {
    std::lock_guard<std::mutex> lock(hardware_mutex_);

    if (i2c_fd_ < 0) {
        return false;
    }

    // 读取 WHO_AM_I 寄存器验证设备
    int who_am_i = ReadRegister(MPU6050_WHO_AM_I);
    return (who_am_i == MPU6050_WHO_AM_I_VALUE);
}

bool Mpu6050Driver::Initialize() {
    std::lock_guard<std::mutex> lock(hardware_mutex_);

    // 先调用基类初始化
    if (!ImuBase::Initialize()) {
        return false;
    }

    // 保存 frame_id 映射（从基类的配置中获取）
    // 注意：基类的 imu_configs_ 是 protected，可以直接访问
    for (const auto& pair : imu_configs_) {
        sensor_frame_ids_[pair.first] = pair.second.frame_id();
    }

    // 初始化硬件
    if (!InitializeHardware()) {
        AERROR << "Failed to initialize MPU6050 hardware";
        return false;
    }

    hardware_initialized_ = true;
    AINFO << "MPU6050 driver initialized successfully";
    return true;
}

void Mpu6050Driver::Cleanup() {
    std::lock_guard<std::mutex> lock(hardware_mutex_);

    // 调用基类清理
    ImuBase::Cleanup();

    // 关闭 I2C 设备
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }

    hardware_initialized_ = false;

    // 清理 frame_id 映射
    sensor_frame_ids_.clear();

    AINFO << "MPU6050 driver cleaned up";
}

bool Mpu6050Driver::InitializeHardware() {
    // 打开 I2C 设备
    i2c_fd_ = open(i2c_device_path_.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        AERROR << "Failed to open I2C device: " << i2c_device_path_
               << " (error: " << strerror(errno) << ")";
        return false;
    }

    // 设置 I2C 从设备地址
    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
        AERROR << "Failed to set I2C slave address: "
               << static_cast<int>(i2c_address_)
               << " (error: " << strerror(errno) << ")";
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // 读取 WHO_AM_I 寄存器验证设备
    int who_am_i = ReadRegister(MPU6050_WHO_AM_I);
    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        AERROR << "MPU6050 WHO_AM_I mismatch: expected "
               << static_cast<int>(MPU6050_WHO_AM_I_VALUE) << ", got "
               << who_am_i;
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // 复位设备
    if (!WriteRegister(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_RESET)) {
        AERROR << "Failed to reset MPU6050";
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // 等待复位完成（约 100ms）
    usleep(100000);

    // 唤醒设备
    if (!WriteRegister(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_WAKEUP)) {
        AERROR << "Failed to wake up MPU6050";
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // 配置采样率分频器（1kHz / (1 + 4) = 200Hz）
    if (!WriteRegister(MPU6050_SMPLRT_DIV, 4)) {
        AERROR << "Failed to set sample rate divider";
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // 配置低通滤波器（DLPF_CFG = 2, 带宽 94Hz）
    if (!WriteRegister(MPU6050_CONFIG, 0x02)) {
        AERROR << "Failed to set DLPF config";
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // 配置陀螺仪量程（±2000°/s）
    uint8_t gyro_config = 0x18;  // FS_SEL = 3 (2000°/s)
    if (!WriteRegister(MPU6050_GYRO_CONFIG, gyro_config)) {
        AERROR << "Failed to set gyro config";
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // 配置加速度计量程（±16g）
    uint8_t accel_config = 0x18;  // AFS_SEL = 3 (16g)
    if (!WriteRegister(MPU6050_ACCEL_CONFIG, accel_config)) {
        AERROR << "Failed to set accel config";
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    AINFO << "MPU6050 hardware initialized successfully";
    return true;
}

int Mpu6050Driver::ReadRegister(uint8_t reg) const {
    if (i2c_fd_ < 0) {
        return -1;
    }

    uint8_t value = 0;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    // 写寄存器地址
    messages[0].addr = i2c_address_;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = const_cast<uint8_t*>(&reg);

    // 读寄存器值
    messages[1].addr = i2c_address_;
    messages[1].flags = I2C_M_RD;
    messages[1].len = 1;
    messages[1].buf = &value;

    packets.msgs = messages;
    packets.nmsgs = 2;

    if (ioctl(i2c_fd_, I2C_RDWR, &packets) < 0) {
        AERROR << "Failed to read register 0x" << std::hex
               << static_cast<int>(reg) << std::dec
               << " (error: " << strerror(errno) << ")";
        return -1;
    }

    return value;
}

bool Mpu6050Driver::WriteRegister(uint8_t reg, uint8_t value) {
    if (i2c_fd_ < 0) {
        return false;
    }

    uint8_t buffer[2] = {reg, value};

    if (write(i2c_fd_, buffer, 2) != 2) {
        AERROR << "Failed to write register 0x" << std::hex
               << static_cast<int>(reg) << std::dec
               << " (error: " << strerror(errno) << ")";
        return false;
    }

    return true;
}

bool Mpu6050Driver::ReadAccelerometer(int16_t& accel_x, int16_t& accel_y,
                                      int16_t& accel_z) {
    if (i2c_fd_ < 0) {
        return false;
    }

    uint8_t buffer[6];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    // 写寄存器地址
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    messages[0].addr = i2c_address_;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &reg;

    // 读 6 字节数据
    messages[1].addr = i2c_address_;
    messages[1].flags = I2C_M_RD;
    messages[1].len = 6;
    messages[1].buf = buffer;

    packets.msgs = messages;
    packets.nmsgs = 2;

    if (ioctl(i2c_fd_, I2C_RDWR, &packets) < 0) {
        AERROR << "Failed to read accelerometer data (error: "
               << strerror(errno) << ")";
        return false;
    }

    // 转换字节序（大端序）
    accel_x = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
    accel_y = (static_cast<int16_t>(buffer[2]) << 8) | buffer[3];
    accel_z = (static_cast<int16_t>(buffer[4]) << 8) | buffer[5];

    return true;
}

bool Mpu6050Driver::ReadGyroscope(int16_t& gyro_x, int16_t& gyro_y,
                                  int16_t& gyro_z) {
    if (i2c_fd_ < 0) {
        return false;
    }

    uint8_t buffer[6];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    // 写寄存器地址
    uint8_t reg = MPU6050_GYRO_XOUT_H;
    messages[0].addr = i2c_address_;
    messages[0].flags = 0;
    messages[0].len = 1;
    messages[0].buf = &reg;

    // 读 6 字节数据
    messages[1].addr = i2c_address_;
    messages[1].flags = I2C_M_RD;
    messages[1].len = 6;
    messages[1].buf = buffer;

    packets.msgs = messages;
    packets.nmsgs = 2;

    if (ioctl(i2c_fd_, I2C_RDWR, &packets) < 0) {
        AERROR << "Failed to read gyroscope data (error: " << strerror(errno)
               << ")";
        return false;
    }

    // 转换字节序（大端序）
    gyro_x = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
    gyro_y = (static_cast<int16_t>(buffer[2]) << 8) | buffer[3];
    gyro_z = (static_cast<int16_t>(buffer[4]) << 8) | buffer[5];

    return true;
}

double Mpu6050Driver::ConvertAccelerometer(int16_t raw_value, int range) {
    // MPU6050 加速度计分辨率：16-bit，量程为 ±range g
    // 转换公式：value = (raw_value / 32768.0) * range * 9.80665
    constexpr double GRAVITY = 9.80665;  // m/s²
    return (static_cast<double>(raw_value) / 32768.0) *
           static_cast<double>(range) * GRAVITY;
}

double Mpu6050Driver::ConvertGyroscope(int16_t raw_value, int range) {
    // MPU6050 陀螺仪分辨率：16-bit，量程为 ±range °/s
    // 转换公式：value = (raw_value / 32768.0) * range * (π / 180.0)
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    return (static_cast<double>(raw_value) / 32768.0) *
           static_cast<double>(range) * DEG_TO_RAD;
}

std::shared_ptr<commsgs::sensor_msgs::Imu> Mpu6050Driver::ReadImuData(
    const std::string& sensor_id) {
    std::lock_guard<std::mutex> lock(hardware_mutex_);

    if (!hardware_initialized_ || i2c_fd_ < 0) {
        return nullptr;
    }

    // 读取加速度计数据
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    if (!ReadAccelerometer(accel_x_raw, accel_y_raw, accel_z_raw)) {
        return nullptr;
    }

    // 读取陀螺仪数据
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    if (!ReadGyroscope(gyro_x_raw, gyro_y_raw, gyro_z_raw)) {
        return nullptr;
    }

    // 转换为物理单位
    double accel_x = ConvertAccelerometer(accel_x_raw, accel_range_);
    double accel_y = ConvertAccelerometer(accel_y_raw, accel_range_);
    double accel_z = ConvertAccelerometer(accel_z_raw, accel_range_);

    double gyro_x = ConvertGyroscope(gyro_x_raw, gyro_range_);
    double gyro_y = ConvertGyroscope(gyro_y_raw, gyro_range_);
    double gyro_z = ConvertGyroscope(gyro_z_raw, gyro_range_);

    // 创建 IMU 消息
    auto imu_msg = std::make_shared<commsgs::sensor_msgs::Imu>();

    // 设置时间戳
    imu_msg->header.stamp = commsgs::builtin_interfaces::Time::Now();

    // 从配置中获取 frame_id
    auto frame_it = sensor_frame_ids_.find(sensor_id);
    if (frame_it != sensor_frame_ids_.end()) {
        imu_msg->header.frame_id = frame_it->second;
    } else {
        imu_msg->header.frame_id = "imu_link";
    }

    // 设置线性加速度（m/s²）
    imu_msg->linear_acceleration.x = accel_x;
    imu_msg->linear_acceleration.y = accel_y;
    imu_msg->linear_acceleration.z = accel_z;

    // 设置角速度（rad/s）
    imu_msg->angular_velocity.x = gyro_x;
    imu_msg->angular_velocity.y = gyro_y;
    imu_msg->angular_velocity.z = gyro_z;

    // 设置方向（MPU6050 没有磁力计，方向需要融合算法计算，这里设为单位四元数）
    imu_msg->orientation.w = 1.0;
    imu_msg->orientation.x = 0.0;
    imu_msg->orientation.y = 0.0;
    imu_msg->orientation.z = 0.0;

    // 设置协方差矩阵（默认值，可以通过配置调整）
    // 线性加速度协方差（对角线元素）
    double accel_cov = 0.01;  // 默认 0.01 m²/s⁴
    imu_msg->linear_acceleration_covariance[0] = accel_cov;
    imu_msg->linear_acceleration_covariance[4] = accel_cov;
    imu_msg->linear_acceleration_covariance[8] = accel_cov;

    // 角速度协方差（对角线元素）
    double gyro_cov = 0.01;  // 默认 0.01 rad²/s²
    imu_msg->angular_velocity_covariance[0] = gyro_cov;
    imu_msg->angular_velocity_covariance[4] = gyro_cov;
    imu_msg->angular_velocity_covariance[8] = gyro_cov;

    // 方向协方差（对角线元素，单位四元数时设为 -1 表示未知）
    imu_msg->orientation_covariance[0] = -1.0;
    imu_msg->orientation_covariance[4] = -1.0;
    imu_msg->orientation_covariance[8] = -1.0;

    return imu_msg;
}

}  // namespace imu
}  // namespace sensor
}  // namespace driver
}  // namespace autonomy

// 注册驱动类为插件，支持动态库加载
CLASS_LOADER_REGISTER_CLASS(autonomy::driver::sensor::imu::Mpu6050Driver,
                            autonomy::driver::sensor::imu::ImuBase)

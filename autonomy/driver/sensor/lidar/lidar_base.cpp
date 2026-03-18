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

#include "autonomy/driver/sensor/lidar/lidar_base.hpp"

#include <chrono>
#include <thread>

#include "autolink/common/log.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace driver {
namespace sensor {
namespace lidar {

using Time = autonomy::common::Time;

// 激光扫描数据包装类，继承 sensor::Data
class LaserScanSensorData : public autonomy::sensor::Data {
 public:
  LaserScanSensorData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& msg)
      : autonomy::sensor::Data(sensor_id), message_(msg) {}

  Time GetTime() const override {
    // 将 builtin_interfaces::Time 转换为 common::Time
    int64_t total_ns = static_cast<int64_t>(message_->header.stamp.sec) * 1000000000LL +
                       static_cast<int64_t>(message_->header.stamp.nanosec);
    return autonomy::common::FromUniversal(total_ns / 100);  // 转换为 100ns ticks
  }

  void AddToCostmap(map::common::MapInterface* costmap_builder) override {
    (void)costmap_builder;
    // 激光扫描数据可以添加到 costmap（待实现）
  }

  const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& GetMessage() const { return message_; }

 private:
  std::shared_ptr<commsgs::sensor_msgs::LaserScan> message_;
};

// 点云数据包装类，继承 sensor::Data
class PointCloudSensorData : public autonomy::sensor::Data {
 public:
  PointCloudSensorData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& msg)
      : autonomy::sensor::Data(sensor_id), message_(msg) {}

  Time GetTime() const override {
    // 将 builtin_interfaces::Time 转换为 common::Time
    int64_t total_ns = static_cast<int64_t>(message_->header.stamp.sec) * 1000000000LL +
                       static_cast<int64_t>(message_->header.stamp.nanosec);
    return autonomy::common::FromUniversal(total_ns / 100);  // 转换为 100ns ticks
  }

  void AddToCostmap(map::common::MapInterface* costmap_builder) override {
    (void)costmap_builder;
    // 点云数据可以添加到 costmap（待实现）
  }

  const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& GetMessage() const { return message_; }

 private:
  std::shared_ptr<commsgs::sensor_msgs::PointCloud2> message_;
};

LidarBase::LidarBase() : stop_thread_(false) {}

LidarBase::~LidarBase() {
  Stop();
  Cleanup();
}

bool LidarBase::Configure(const std::string& name, const proto::DriverOptions& options) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (configured_) {
    AWARN << "LidarBase already configured: " << name_;
    return true;
  }

  name_ = name;

  // 从 DriverOptions 中提取激光雷达配置
  lidar_configs_.clear();
  for (const auto& lidar_option : options.lidars()) {
    if (lidar_option.enabled()) {
      lidar_configs_[lidar_option.sensor_id()] = lidar_option;
      AINFO << "Configured lidar sensor: " << lidar_option.sensor_id() << " (hardware: " << GetHardwareModel() << ")";
    }
  }

  if (lidar_configs_.empty()) {
    AWARN << "No enabled lidar sensors found in configuration";
    return false;
  }

  configured_ = true;
  AINFO << "LidarBase configured: " << name_ << " with " << lidar_configs_.size() << " lidar sensors";
  return true;
}

bool LidarBase::Initialize() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!configured_) {
    AERROR << "LidarBase not configured. Call Configure() first.";
    return false;
  }

  if (initialized_) {
    AWARN << "LidarBase already initialized: " << name_;
    return true;
  }

  // 检查硬件连接
  if (!IsConnected()) {
    AERROR << "Lidar hardware not connected: " << GetHardwareModel();
    return false;
  }

  // 初始化每个传感器
  for (const auto& pair : lidar_configs_) {
    AINFO << "Initialized lidar sensor: " << pair.first;
  }

  initialized_ = true;
  AINFO << "LidarBase initialized: " << name_;
  return true;
}

void LidarBase::Start() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_) {
    AERROR << "LidarBase not initialized. Call Initialize() first.";
    return;
  }

  if (started_) {
    AWARN << "LidarBase already started: " << name_;
    return;
  }

  stop_thread_ = false;
  data_reading_thread_ = std::thread(&LidarBase::DataReadingThread, this);

  started_ = true;
  AINFO << "LidarBase started: " << name_;
}

void LidarBase::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!started_) {
    return;
  }

  stop_thread_ = true;

  if (data_reading_thread_.joinable()) {
    data_reading_thread_.join();
  }

  started_ = false;
  AINFO << "LidarBase stopped: " << name_;
}

void LidarBase::Cleanup() {
  std::lock_guard<std::mutex> lock(mutex_);

  Stop();

  handlers_.clear();
  lidar_configs_.clear();

  configured_ = false;
  initialized_ = false;
  AINFO << "LidarBase cleaned up: " << name_;
}

std::string LidarBase::GetName() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return name_;
}

std::vector<std::string> LidarBase::GetSensorIds() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> sensor_ids;
  for (const auto& pair : lidar_configs_) {
    sensor_ids.push_back(pair.first);
  }
  return sensor_ids;
}

bool LidarBase::IsSensorRegistered(const std::string& sensor_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return lidar_configs_.find(sensor_id) != lidar_configs_.end();
}

bool LidarBase::RegisterSensorHandler(
    const std::string& sensor_id,
    std::function<void(const std::string&, const std::shared_ptr<autonomy::sensor::Data>&)> handler) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!IsSensorRegistered(sensor_id)) {
    AERROR << "Sensor not registered: " << sensor_id;
    return false;
  }

  if (!handler) {
    AERROR << "Handler is null for sensor: " << sensor_id;
    return false;
  }

  handlers_[sensor_id] = handler;
  AINFO << "Registered sensor handler for lidar: " << sensor_id;
  return true;
}

void LidarBase::UnregisterSensorHandler(const std::string& sensor_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = handlers_.find(sensor_id);
  if (it != handlers_.end()) {
    handlers_.erase(it);
    AINFO << "Unregistered sensor handler for lidar: " << sensor_id;
  }
}

void LidarBase::ProcessLaserScanData(const std::string& sensor_id,
                                     const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& scan_msg) {
  if (scan_msg == nullptr) {
    AERROR << "Received null laser scan message for sensor: " << sensor_id;
    return;
  }

  // 转换为 sensor::Data 并转发
  auto sensor_data = std::make_shared<LaserScanSensorData>(sensor_id, scan_msg);

  std::lock_guard<std::mutex> lock(mutex_);
  auto handler_it = handlers_.find(sensor_id);
  if (handler_it != handlers_.end()) {
    handler_it->second(sensor_id, sensor_data);
  } else {
    ADEBUG << "No handler registered for lidar sensor: " << sensor_id;
  }
}

void LidarBase::ProcessPointCloudData(const std::string& sensor_id,
                                      const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& cloud_msg) {
  if (cloud_msg == nullptr) {
    AERROR << "Received null point cloud message for sensor: " << sensor_id;
    return;
  }

  // 转换为 sensor::Data 并转发
  auto sensor_data = std::make_shared<PointCloudSensorData>(sensor_id, cloud_msg);

  std::lock_guard<std::mutex> lock(mutex_);
  auto handler_it = handlers_.find(sensor_id);
  if (handler_it != handlers_.end()) {
    handler_it->second(sensor_id, sensor_data);
  } else {
    ADEBUG << "No handler registered for lidar sensor: " << sensor_id;
  }
}

void LidarBase::DataReadingThread() {
  AINFO << "Lidar data reading thread started for: " << name_;

  // 默认读取间隔（激光雷达通常有自己的扫描频率）
  std::chrono::milliseconds read_interval(100);  // 默认 10Hz

  while (!stop_thread_) {
    std::vector<std::string> sensor_ids;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& pair : lidar_configs_) {
        sensor_ids.push_back(pair.first);
      }
    }

    // 读取每个传感器的数据
    for (const auto& sensor_id : sensor_ids) {
      std::lock_guard<std::mutex> lock(mutex_);
      auto config_it = lidar_configs_.find(sensor_id);
      if (config_it == lidar_configs_.end()) {
        continue;
      }

      const auto& config = config_it->second;

      // 根据激光雷达类型读取相应的数据
      if (config.type() == proto::LidarType::LASER_SCAN) {
        auto scan_msg = ReadLaserScanData(sensor_id);
        if (scan_msg != nullptr) {
          ProcessLaserScanData(sensor_id, scan_msg);
        }
      } else if (config.type() == proto::LidarType::POINT_CLOUD2 || config.type() == proto::LidarType::POINT_CLOUD) {
        auto cloud_msg = ReadPointCloudData(sensor_id);
        if (cloud_msg != nullptr) {
          ProcessPointCloudData(sensor_id, cloud_msg);
        }
      }
    }

    // 等待下一次读取
    std::this_thread::sleep_for(read_interval);
  }

  AINFO << "Lidar data reading thread stopped for: " << name_;
}

}  // namespace lidar
}  // namespace sensor
}  // namespace driver
}  // namespace autonomy

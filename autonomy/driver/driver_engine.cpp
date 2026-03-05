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

#include "autonomy/driver/driver_engine.hpp"

#include <map>

#include "autolink/common/log.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/driver/common/driver_interface.hpp"
#include "autonomy/map/common/map_interface.hpp"

namespace autonomy {
namespace driver {

DriverEngine::DriverEngine(::autolink::Node* node) : node_(node) {
  if (node_ == nullptr) {
    AERROR << "DriverEngine: node is null";
  }
}

DriverEngine::~DriverEngine() { Stop(); }

bool DriverEngine::Initialize(const proto::DriverOptions& options) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (initialized_) {
    AWARN << "DriverEngine already initialized";
    return true;
  }

  if (node_ == nullptr) {
    AERROR << "DriverEngine: node is null, cannot initialize";
    return false;
  }

  // 从 options 加载传感器配置
  // 根据配置决定使用 ROS2 订阅还是硬件驱动

  // 配置激光雷达
  for (const auto& lidar : options.lidars()) {
    if (lidar.enabled()) {
      // 如果配置了 topic，使用 ROS2 模式；否则需要注册硬件驱动
      if (!lidar.topic().empty()) {
        // ROS2 模式：订阅主题
        std::string sensor_type;
        switch (lidar.type()) {
          case proto::LidarType::LASER_SCAN:
            sensor_type = "laser_scan";
            break;
          case proto::LidarType::POINT_CLOUD2:
            sensor_type = "point_cloud2";
            break;
          case proto::LidarType::POINT_CLOUD:
            sensor_type = "point_cloud";
            break;
          case proto::LidarType::MULTI_ECHO_LASER_SCAN:
            sensor_type = "multi_echo_laser_scan";
            break;
          default:
            AWARN << "Unknown lidar type for sensor: " << lidar.sensor_id();
            continue;
        }
        if (!SubscribeSensor(lidar.sensor_id(), lidar.topic(), sensor_type)) {
          AERROR << "Failed to subscribe lidar: " << lidar.sensor_id();
          return false;
        }
        sensor_data_sources_[lidar.sensor_id()] = SensorDataSource::ROS2;
      } else {
        // Driver 模式：需要外部注册硬件驱动
        sensor_data_sources_[lidar.sensor_id()] = SensorDataSource::DRIVER;
        AINFO << "Lidar " << lidar.sensor_id() << " configured for hardware driver mode";
      }
    }
  }

  // 配置IMU
  for (const auto& imu : options.imus()) {
    if (imu.enabled()) {
      if (!imu.topic().empty()) {
        // ROS2 模式
        if (!SubscribeSensor(imu.sensor_id(), imu.topic(), "imu")) {
          AERROR << "Failed to subscribe IMU: " << imu.sensor_id();
          return false;
        }
        sensor_data_sources_[imu.sensor_id()] = SensorDataSource::ROS2;
      } else {
        // Driver 模式
        sensor_data_sources_[imu.sensor_id()] = SensorDataSource::DRIVER;
        AINFO << "IMU " << imu.sensor_id() << " configured for hardware driver mode";
      }
    }
  }

  // 配置相机
  for (const auto& camera : options.cameras()) {
    if (camera.enabled()) {
      if (!camera.image_topic().empty()) {
        // ROS2 模式
        if (!SubscribeSensor(camera.sensor_id(), camera.image_topic(), "image")) {
          AERROR << "Failed to subscribe camera: " << camera.sensor_id();
          return false;
        }
        sensor_data_sources_[camera.sensor_id()] = SensorDataSource::ROS2;
      } else {
        // Driver 模式
        sensor_data_sources_[camera.sensor_id()] = SensorDataSource::DRIVER;
        AINFO << "Camera " << camera.sensor_id() << " configured for hardware driver mode";
      }
    }
  }

  // 配置测距传感器
  for (const auto& range : options.ranges()) {
    if (range.enabled()) {
      if (!range.topic().empty()) {
        // ROS2 模式
        if (!SubscribeSensor(range.sensor_id(), range.topic(), "range")) {
          AERROR << "Failed to subscribe range sensor: " << range.sensor_id();
          return false;
        }
        sensor_data_sources_[range.sensor_id()] = SensorDataSource::ROS2;
      } else {
        // Driver 模式
        sensor_data_sources_[range.sensor_id()] = SensorDataSource::DRIVER;
        AINFO << "Range sensor " << range.sensor_id() << " configured for hardware driver mode";
      }
    }
  }

  // 配置GPS传感器
  for (const auto& gps : options.gps_sensors()) {
    if (gps.enabled()) {
      if (!gps.topic().empty()) {
        // TODO: 需要定义 NavSatFix 消息类型后实现
        AWARN << "GPS sensor ROS2 subscription not yet implemented: " << gps.sensor_id();
        sensor_data_sources_[gps.sensor_id()] = SensorDataSource::ROS2;
      } else {
        // Driver 模式
        sensor_data_sources_[gps.sensor_id()] = SensorDataSource::DRIVER;
        AINFO << "GPS sensor " << gps.sensor_id() << " configured for hardware driver mode";
      }
    }
  }

  initialized_ = true;
  AINFO << "DriverEngine initialized with " << options.lidars_size() << " lidars, " << options.imus_size() << " IMUs, "
        << options.cameras_size() << " cameras, " << options.ranges_size() << " range sensors, "
        << options.gps_sensors_size() << " GPS sensors";
  return true;
}

void DriverEngine::Start() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_) {
    AERROR << "DriverEngine not initialized. Call Initialize() first.";
    return;
  }

  if (started_) {
    AWARN << "DriverEngine already started";
    return;
  }

  // 启动所有硬件驱动
  for (auto& pair : hardware_drivers_) {
    if (pair.second != nullptr) {
      pair.second->Start();
      AINFO << "Started hardware driver for sensor: " << pair.first;
    }
  }

  started_ = true;
  AINFO << "DriverEngine started";
}

void DriverEngine::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!started_) {
    return;
  }

  // 停止所有硬件驱动
  for (auto& pair : hardware_drivers_) {
    if (pair.second != nullptr) {
      pair.second->Stop();
      AINFO << "Stopped hardware driver for sensor: " << pair.first;
    }
  }

  // 取消订阅所有 ROS2 传感器
  UnsubscribeAllSensors();

  started_ = false;
  AINFO << "DriverEngine stopped";
}

bool DriverEngine::RegisterSensorHandler(const std::string& sensor_id, SensorDataHandler handler) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (sensor_id.empty()) {
    AERROR << "Sensor ID is empty";
    return false;
  }

  if (!handler) {
    AERROR << "Sensor handler is null for sensor: " << sensor_id;
    return false;
  }

  sensor_handlers_.insert(std::make_pair(sensor_id, handler));
  AINFO << "Registered sensor handler for sensor: " << sensor_id;
  return true;
}

void DriverEngine::UnregisterSensorHandler(const std::string& sensor_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = sensor_handlers_.find(sensor_id);
  if (it != sensor_handlers_.end()) {
    sensor_handlers_.erase(it);
    AINFO << "Unregistered sensor handler for sensor: " << sensor_id;
  }
}

bool DriverEngine::SubscribeSensor(const std::string& sensor_id, const std::string& topic,
                                   const std::string& sensor_type) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (sensor_id.empty() || topic.empty() || sensor_type.empty()) {
    AERROR << "Invalid sensor subscription parameters";
    return false;
  }

  if (node_ == nullptr) {
    AERROR << "DriverEngine: node is null, cannot subscribe";
    return false;
  }

  // 检查是否已经订阅
  if (sensor_subscriptions_.find(sensor_id) != sensor_subscriptions_.end()) {
    AWARN << "Sensor already subscribed: " << sensor_id;
    return false;
  }

  SensorSubscription subscription;
  subscription.sensor_id = sensor_id;
  subscription.topic = topic;
  subscription.sensor_type = sensor_type;

  // 根据传感器类型创建订阅器
  if (sensor_type == "laser_scan" || sensor_type == "LaserScan") {
    auto callback = [this, sensor_id](const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& msg) {
      this->AddSensorData(sensor_id, msg);
    };
    auto reader = node_->CreateReader<commsgs::sensor_msgs::LaserScan>(
        topic, ::autolink::CallbackFunc<commsgs::sensor_msgs::LaserScan>(callback));
    if (reader == nullptr) {
      AERROR << "Failed to create LaserScan reader for topic: " << topic;
      return false;
    }
    subscription.reader = std::static_pointer_cast<void>(reader);
  } else if (sensor_type == "point_cloud2" || sensor_type == "PointCloud2") {
    auto callback = [this, sensor_id](const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& msg) {
      this->AddSensorData(sensor_id, msg);
    };
    auto reader = node_->CreateReader<commsgs::sensor_msgs::PointCloud2>(
        topic, ::autolink::CallbackFunc<commsgs::sensor_msgs::PointCloud2>(callback));
    if (reader == nullptr) {
      AERROR << "Failed to create PointCloud2 reader for topic: " << topic;
      return false;
    }
    subscription.reader = std::static_pointer_cast<void>(reader);
  } else if (sensor_type == "point_cloud" || sensor_type == "PointCloud") {
    auto callback = [this, sensor_id](const std::shared_ptr<commsgs::sensor_msgs::PointCloud>& msg) {
      this->AddSensorData(sensor_id, msg);
    };
    auto reader = node_->CreateReader<commsgs::sensor_msgs::PointCloud>(
        topic, ::autolink::CallbackFunc<commsgs::sensor_msgs::PointCloud>(callback));
    if (reader == nullptr) {
      AERROR << "Failed to create PointCloud reader for topic: " << topic;
      return false;
    }
    subscription.reader = std::static_pointer_cast<void>(reader);
  } else if (sensor_type == "multi_echo_laser_scan" || sensor_type == "MultiEchoLaserScan") {
    // MultiEchoLaserScan 使用与 LaserScan 相同的处理方式
    auto callback = [this, sensor_id](const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& msg) {
      this->AddSensorData(sensor_id, msg);
    };
    auto reader = node_->CreateReader<commsgs::sensor_msgs::LaserScan>(
        topic, ::autolink::CallbackFunc<commsgs::sensor_msgs::LaserScan>(callback));
    if (reader == nullptr) {
      AERROR << "Failed to create MultiEchoLaserScan reader for topic: " << topic;
      return false;
    }
    subscription.reader = std::static_pointer_cast<void>(reader);
  } else if (sensor_type == "imu" || sensor_type == "Imu") {
    auto callback = [this, sensor_id](const std::shared_ptr<commsgs::sensor_msgs::Imu>& msg) {
      this->AddSensorData(sensor_id, msg);
    };
    auto reader = node_->CreateReader<commsgs::sensor_msgs::Imu>(
        topic, ::autolink::CallbackFunc<commsgs::sensor_msgs::Imu>(callback));
    if (reader == nullptr) {
      AERROR << "Failed to create Imu reader for topic: " << topic;
      return false;
    }
    subscription.reader = std::static_pointer_cast<void>(reader);
  } else if (sensor_type == "odometry" || sensor_type == "Odometry") {
    auto callback = [this, sensor_id](const std::shared_ptr<commsgs::planning_msgs::Odometry>& msg) {
      this->AddSensorData(sensor_id, msg);
    };
    auto reader = node_->CreateReader<commsgs::planning_msgs::Odometry>(
        topic, ::autolink::CallbackFunc<commsgs::planning_msgs::Odometry>(callback));
    if (reader == nullptr) {
      AERROR << "Failed to create Odometry reader for topic: " << topic;
      return false;
    }
    subscription.reader = std::static_pointer_cast<void>(reader);
  } else if (sensor_type == "image" || sensor_type == "Image") {
    auto callback = [this, sensor_id](const std::shared_ptr<commsgs::sensor_msgs::Image>& msg) {
      this->AddSensorData(sensor_id, msg);
    };
    auto reader = node_->CreateReader<commsgs::sensor_msgs::Image>(
        topic, ::autolink::CallbackFunc<commsgs::sensor_msgs::Image>(callback));
    if (reader == nullptr) {
      AERROR << "Failed to create Image reader for topic: " << topic;
      return false;
    }
    subscription.reader = std::static_pointer_cast<void>(reader);
  } else if (sensor_type == "range" || sensor_type == "Range") {
    auto callback = [this, sensor_id](const std::shared_ptr<commsgs::sensor_msgs::Range>& msg) {
      this->AddSensorData(sensor_id, msg);
    };
    auto reader = node_->CreateReader<commsgs::sensor_msgs::Range>(
        topic, ::autolink::CallbackFunc<commsgs::sensor_msgs::Range>(callback));
    if (reader == nullptr) {
      AERROR << "Failed to create Range reader for topic: " << topic;
      return false;
    }
    subscription.reader = std::static_pointer_cast<void>(reader);
  } else if (sensor_type == "nav_sat_fix" || sensor_type == "NavSatFix") {
    // TODO: 需要定义 NavSatFix 消息类型后实现
    AERROR << "NavSatFix sensor type not yet supported. Please define "
              "NavSatFix message type first.";
    return false;
  } else {
    AERROR << "Unsupported sensor type: " << sensor_type;
    return false;
  }

  sensor_subscriptions_[sensor_id] = subscription;
  AINFO << "Subscribed to sensor: " << sensor_id << " on topic: " << topic << " with type: " << sensor_type;
  return true;
}

void DriverEngine::UnsubscribeSensor(const std::string& sensor_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = sensor_subscriptions_.find(sensor_id);
  if (it != sensor_subscriptions_.end()) {
    sensor_subscriptions_.erase(it);
    AINFO << "Unsubscribed from sensor: " << sensor_id;
  }
}

void DriverEngine::UnsubscribeAllSensors() {
  std::lock_guard<std::mutex> lock(mutex_);

  sensor_subscriptions_.clear();
  AINFO << "Unsubscribed from all sensors";
}

void DriverEngine::AddSensorData(const std::string& sensor_id,
                                 const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& message) {
  if (message == nullptr) {
    AERROR << "Received null LaserScan message for sensor: " << sensor_id;
    return;
  }

  // 检查数据源类型（需要先检查，避免在 Driver 模式下处理 ROS2 数据）
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sensor_data_sources_.find(sensor_id);
    if (it != sensor_data_sources_.end() && it->second == SensorDataSource::DRIVER) {
      AWARN << "Sensor " << sensor_id
            << " is configured for hardware driver mode, ignoring ROS2 "
               "data";
      return;
    }
  }

  ADEBUG << "Received LaserScan from sensor: " << sensor_id << " with " << message->ranges.size() << " ranges";

  // 将 LaserScan 转换为 sensor::Data 并转发
  class LaserScanData : public sensor::Data {
   public:
    LaserScanData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& msg)
        : Data(sensor_id), message_(msg) {}
    autonomy::common::Time GetTime() const override {
      // 将 builtin_interfaces::Time 转换为 common::Time
      int64_t total_ns = static_cast<int64_t>(message_->header.stamp.sec) * 1000000000LL +
                         static_cast<int64_t>(message_->header.stamp.nanosec);
      return autonomy::common::FromUniversal(total_ns / 100);  // 转换为 100ns ticks
    }
    void AddToCostmap(map::common::MapInterface* costmap_builder) override {
      (void)costmap_builder;
      // TODO: 实现添加到 costmap 的逻辑
    }
    const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& GetMessage() const { return message_; }

   private:
    std::shared_ptr<commsgs::sensor_msgs::LaserScan> message_;
  };

  auto sensor_data = std::make_shared<LaserScanData>(sensor_id, message);
  ForwardSensorData(sensor_id, sensor_data);
}

void DriverEngine::AddSensorData(const std::string& sensor_id,
                                 const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& message) {
  if (message == nullptr) {
    AERROR << "Received null PointCloud2 message for sensor: " << sensor_id;
    return;
  }

  // 检查数据源类型
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sensor_data_sources_.find(sensor_id);
    if (it != sensor_data_sources_.end() && it->second == SensorDataSource::DRIVER) {
      AWARN << "Sensor " << sensor_id
            << " is configured for hardware driver mode, ignoring ROS2 "
               "data";
      return;
    }
  }

  ADEBUG << "Received PointCloud2 from sensor: " << sensor_id << " with width: " << message->width
         << ", height: " << message->height;

  // 将 PointCloud2 转换为 sensor::Data 并转发
  class PointCloud2Data : public sensor::Data {
   public:
    PointCloud2Data(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& msg)
        : Data(sensor_id), message_(msg) {}
    autonomy::common::Time GetTime() const override {
      // 将 builtin_interfaces::Time 转换为 common::Time
      int64_t total_ns = static_cast<int64_t>(message_->header.stamp.sec) * 1000000000LL +
                         static_cast<int64_t>(message_->header.stamp.nanosec);
      return autonomy::common::FromUniversal(total_ns / 100);  // 转换为 100ns ticks
    }
    void AddToCostmap(map::common::MapInterface* costmap_builder) override {
      (void)costmap_builder;
      // TODO: 实现添加到 costmap 的逻辑
    }
    const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& GetMessage() const { return message_; }

   private:
    std::shared_ptr<commsgs::sensor_msgs::PointCloud2> message_;
  };

  auto sensor_data = std::make_shared<PointCloud2Data>(sensor_id, message);
  ForwardSensorData(sensor_id, sensor_data);
}

void DriverEngine::AddSensorData(const std::string& sensor_id,
                                 const std::shared_ptr<commsgs::sensor_msgs::PointCloud>& message) {
  if (message == nullptr) {
    AERROR << "Received null PointCloud message for sensor: " << sensor_id;
    return;
  }

  // 检查数据源类型
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sensor_data_sources_.find(sensor_id);
    if (it != sensor_data_sources_.end() && it->second == SensorDataSource::DRIVER) {
      AWARN << "Sensor " << sensor_id
            << " is configured for hardware driver mode, ignoring ROS2 "
               "data";
      return;
    }
  }

  ADEBUG << "Received PointCloud from sensor: " << sensor_id << " with " << message->points.size() << " points";

  // 将 PointCloud 转换为 sensor::Data 并转发
  class PointCloudData : public sensor::Data {
   public:
    PointCloudData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::PointCloud>& msg)
        : Data(sensor_id), message_(msg) {}
    autonomy::common::Time GetTime() const override {
      // 将 builtin_interfaces::Time 转换为 common::Time
      int64_t total_ns = static_cast<int64_t>(message_->header.stamp.sec) * 1000000000LL +
                         static_cast<int64_t>(message_->header.stamp.nanosec);
      return autonomy::common::FromUniversal(total_ns / 100);  // 转换为 100ns ticks
    }
    void AddToCostmap(map::common::MapInterface* costmap_builder) override {
      (void)costmap_builder;
      // TODO: 实现添加到 costmap 的逻辑
    }
    const std::shared_ptr<commsgs::sensor_msgs::PointCloud>& GetMessage() const { return message_; }

   private:
    std::shared_ptr<commsgs::sensor_msgs::PointCloud> message_;
  };

  auto sensor_data = std::make_shared<PointCloudData>(sensor_id, message);
  ForwardSensorData(sensor_id, sensor_data);
}

void DriverEngine::AddSensorData(const std::string& sensor_id,
                                 const std::shared_ptr<commsgs::sensor_msgs::Imu>& message) {
  if (message == nullptr) {
    AERROR << "Received null Imu message for sensor: " << sensor_id;
    return;
  }

  // 检查数据源类型
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sensor_data_sources_.find(sensor_id);
    if (it != sensor_data_sources_.end() && it->second == SensorDataSource::DRIVER) {
      AWARN << "Sensor " << sensor_id
            << " is configured for hardware driver mode, ignoring ROS2 "
               "data";
      return;
    }
  }

  ADEBUG << "Received Imu from sensor: " << sensor_id;

  // 将 Imu 转换为 sensor::Data 并转发
  class ImuData : public sensor::Data {
   public:
    ImuData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::Imu>& msg)
        : Data(sensor_id), message_(msg) {}
    autonomy::common::Time GetTime() const override {
      // 将 builtin_interfaces::Time 转换为 common::Time
      int64_t total_ns = static_cast<int64_t>(message_->header.stamp.sec) * 1000000000LL +
                         static_cast<int64_t>(message_->header.stamp.nanosec);
      return autonomy::common::FromUniversal(total_ns / 100);  // 转换为 100ns ticks
    }
    void AddToCostmap(map::common::MapInterface* costmap_builder) override {
      (void)costmap_builder;
      // IMU 数据通常不直接添加到 costmap
    }
    const std::shared_ptr<commsgs::sensor_msgs::Imu>& GetMessage() const { return message_; }

   private:
    std::shared_ptr<commsgs::sensor_msgs::Imu> message_;
  };

  auto sensor_data = std::make_shared<ImuData>(sensor_id, message);
  ForwardSensorData(sensor_id, sensor_data);
}

void DriverEngine::AddSensorData(const std::string& sensor_id,
                                 const std::shared_ptr<commsgs::planning_msgs::Odometry>& message) {
  if (message == nullptr) {
    AERROR << "Received null Odometry message for sensor: " << sensor_id;
    return;
  }

  // 检查数据源类型
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sensor_data_sources_.find(sensor_id);
    if (it != sensor_data_sources_.end() && it->second == SensorDataSource::DRIVER) {
      AWARN << "Sensor " << sensor_id
            << " is configured for hardware driver mode, ignoring ROS2 "
               "data";
      return;
    }
  }

  ADEBUG << "Received Odometry from sensor: " << sensor_id;

  // 将 Odometry 转换为 sensor::Data 并转发
  class OdometryData : public sensor::Data {
   public:
    OdometryData(const std::string& sensor_id, const std::shared_ptr<commsgs::planning_msgs::Odometry>& msg)
        : Data(sensor_id), message_(msg) {}
    autonomy::common::Time GetTime() const override {
      // 将 builtin_interfaces::Time 转换为 common::Time
      int64_t total_ns = static_cast<int64_t>(message_->header.stamp.sec) * 1000000000LL +
                         static_cast<int64_t>(message_->header.stamp.nanosec);
      return autonomy::common::FromUniversal(total_ns / 100);  // 转换为 100ns ticks
    }
    void AddToCostmap(map::common::MapInterface* costmap_builder) override {
      (void)costmap_builder;
      // Odometry 数据通常不直接添加到 costmap
    }
    const std::shared_ptr<commsgs::planning_msgs::Odometry>& GetMessage() const { return message_; }

   private:
    std::shared_ptr<commsgs::planning_msgs::Odometry> message_;
  };

  auto sensor_data = std::make_shared<OdometryData>(sensor_id, message);
  ForwardSensorData(sensor_id, sensor_data);
}

void DriverEngine::AddSensorData(const std::string& sensor_id,
                                 const std::shared_ptr<commsgs::sensor_msgs::Image>& message) {
  if (message == nullptr) {
    AERROR << "Received null Image message for sensor: " << sensor_id;
    return;
  }

  // 检查数据源类型
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sensor_data_sources_.find(sensor_id);
    if (it != sensor_data_sources_.end() && it->second == SensorDataSource::DRIVER) {
      AWARN << "Sensor " << sensor_id
            << " is configured for hardware driver mode, ignoring ROS2 "
               "data";
      return;
    }
  }

  ADEBUG << "Received Image from sensor: " << sensor_id << " with size: " << message->width << "x" << message->height;

  // 将 Image 转换为 sensor::Data 并转发
  class ImageData : public sensor::Data {
   public:
    ImageData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::Image>& msg)
        : Data(sensor_id), message_(msg) {}
    autonomy::common::Time GetTime() const override {
      // 将 builtin_interfaces::Time 转换为 common::Time
      int64_t total_ns = static_cast<int64_t>(message_->header.stamp.sec) * 1000000000LL +
                         static_cast<int64_t>(message_->header.stamp.nanosec);
      return autonomy::common::FromUniversal(total_ns / 100);  // 转换为 100ns ticks
    }
    void AddToCostmap(map::common::MapInterface* costmap_builder) override {
      (void)costmap_builder;
      // TODO: 实现图像数据添加到 costmap 的逻辑（如果需要）
    }
    const std::shared_ptr<commsgs::sensor_msgs::Image>& GetMessage() const { return message_; }

   private:
    std::shared_ptr<commsgs::sensor_msgs::Image> message_;
  };

  auto sensor_data = std::make_shared<ImageData>(sensor_id, message);
  ForwardSensorData(sensor_id, sensor_data);
}

void DriverEngine::AddSensorData(const std::string& sensor_id,
                                 const std::shared_ptr<commsgs::sensor_msgs::Range>& message) {
  if (message == nullptr) {
    AERROR << "Received null Range message for sensor: " << sensor_id;
    return;
  }

  // 检查数据源类型
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = sensor_data_sources_.find(sensor_id);
    if (it != sensor_data_sources_.end() && it->second == SensorDataSource::DRIVER) {
      AWARN << "Sensor " << sensor_id
            << " is configured for hardware driver mode, ignoring ROS2 "
               "data";
      return;
    }
  }

  ADEBUG << "Received Range from sensor: " << sensor_id << " with range: " << message->range
         << " m (min: " << message->min_range << ", max: " << message->max_range << ")";

  // 将 Range 转换为 sensor::Data 并转发
  class RangeData : public sensor::Data {
   public:
    RangeData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::Range>& msg)
        : Data(sensor_id), message_(msg) {}
    autonomy::common::Time GetTime() const override {
      // 将 builtin_interfaces::Time 转换为 common::Time
      int64_t total_ns = static_cast<int64_t>(message_->header.stamp.sec) * 1000000000LL +
                         static_cast<int64_t>(message_->header.stamp.nanosec);
      return autonomy::common::FromUniversal(total_ns / 100);  // 转换为 100ns ticks
    }
    void AddToCostmap(map::common::MapInterface* costmap_builder) override {
      (void)costmap_builder;
      // TODO: 实现测距数据添加到 costmap 的逻辑
    }
    const std::shared_ptr<commsgs::sensor_msgs::Range>& GetMessage() const { return message_; }

   private:
    std::shared_ptr<commsgs::sensor_msgs::Range> message_;
  };

  auto sensor_data = std::make_shared<RangeData>(sensor_id, message);
  ForwardSensorData(sensor_id, sensor_data);
}

void DriverEngine::AddSensorData(const std::string& sensor_id, const std::shared_ptr<void>& message) {
  if (message == nullptr) {
    AERROR << "Received null NavSatFix message for sensor: " << sensor_id;
    return;
  }

  // TODO: 定义 NavSatFix 消息类型并实现转换逻辑
  ADEBUG << "Received NavSatFix from sensor: " << sensor_id;

  // ForwardSensorData(sensor_id, converted_data);
}

std::vector<std::string> DriverEngine::GetRegisteredSensors() const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<std::string> sensors;
  // 收集 ROS2 模式的传感器
  for (const auto& pair : sensor_subscriptions_) {
    sensors.push_back(pair.first);
  }
  // 收集 Driver 模式的传感器
  for (const auto& pair : hardware_drivers_) {
    if (std::find(sensors.begin(), sensors.end(), pair.first) == sensors.end()) {
      sensors.push_back(pair.first);
    }
  }
  return sensors;
}

bool DriverEngine::IsSensorRegistered(const std::string& sensor_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  // 检查是否在 ROS2 订阅或硬件驱动中注册
  return sensor_subscriptions_.find(sensor_id) != sensor_subscriptions_.end() ||
         hardware_drivers_.find(sensor_id) != hardware_drivers_.end();
}

bool DriverEngine::RegisterHardwareDriver(const std::string& sensor_id, common::DriverInterface* driver) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (sensor_id.empty()) {
    AERROR << "Sensor ID is empty";
    return false;
  }

  if (driver == nullptr) {
    AERROR << "Hardware driver is null for sensor: " << sensor_id;
    return false;
  }

  // 检查是否已经注册
  if (hardware_drivers_.find(sensor_id) != hardware_drivers_.end()) {
    AWARN << "Hardware driver already registered for sensor: " << sensor_id;
    return false;
  }

  // 检查数据源类型
  auto it = sensor_data_sources_.find(sensor_id);
  if (it != sensor_data_sources_.end() && it->second != SensorDataSource::DRIVER) {
    AERROR << "Sensor " << sensor_id << " is configured for ROS2 mode, cannot register hardware driver";
    return false;
  }

  // 注册硬件驱动
  hardware_drivers_[sensor_id] = driver;
  sensor_data_sources_[sensor_id] = SensorDataSource::DRIVER;

  // 注册数据处理器，将驱动数据转发到 ForwardSensorData
  auto handler = [this](const std::string& id, const std::shared_ptr<sensor::Data>& data) {
    this->OnHardwareDriverData(id, data);
  };
  if (!driver->RegisterSensorHandler(sensor_id, handler)) {
    AERROR << "Failed to register sensor handler with hardware driver: " << sensor_id;
    hardware_drivers_.erase(sensor_id);
    return false;
  }

  // 如果已启动，立即启动驱动
  if (started_) {
    driver->Start();
  }

  AINFO << "Registered hardware driver for sensor: " << sensor_id;
  return true;
}

void DriverEngine::UnregisterHardwareDriver(const std::string& sensor_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = hardware_drivers_.find(sensor_id);
  if (it != hardware_drivers_.end()) {
    if (it->second != nullptr) {
      it->second->Stop();
      it->second->UnregisterSensorHandler(sensor_id);
    }
    hardware_drivers_.erase(it);
    sensor_data_sources_.erase(sensor_id);
    AINFO << "Unregistered hardware driver for sensor: " << sensor_id;
  }
}

bool DriverEngine::UsesHardwareDriver(const std::string& sensor_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = sensor_data_sources_.find(sensor_id);
  if (it != sensor_data_sources_.end()) {
    return it->second == SensorDataSource::DRIVER;
  }
  return false;
}

std::vector<std::string> DriverEngine::GetHardwareDriverSensors() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> sensors;
  for (const auto& pair : hardware_drivers_) {
    sensors.push_back(pair.first);
  }
  return sensors;
}

void DriverEngine::OnHardwareDriverData(const std::string& sensor_id, const std::shared_ptr<sensor::Data>& data) {
  // 硬件驱动数据到达时的回调
  // 直接转发到注册的处理器
  ForwardSensorData(sensor_id, data);
}

void DriverEngine::ForwardSensorData(const std::string& sensor_id, const std::shared_ptr<sensor::Data>& data) {
  if (data == nullptr) {
    AERROR << "Cannot forward null sensor data for sensor: " << sensor_id;
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // 查找并调用所有注册的处理器
  auto range = sensor_handlers_.equal_range(sensor_id);
  if (range.first == range.second) {
    // 如果没有特定传感器处理器，尝试查找通用处理器（使用空字符串作为key）
    auto it = sensor_handlers_.find("");
    if (it != sensor_handlers_.end()) {
      it->second(sensor_id, data);
    } else {
      ADEBUG << "No handler registered for sensor: " << sensor_id;
    }
  } else {
    for (auto it = range.first; it != range.second; ++it) {
      it->second(sensor_id, data);
    }
  }
}

}  // namespace driver
}  // namespace autonomy

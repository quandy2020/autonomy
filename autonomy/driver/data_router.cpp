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

#include "autonomy/driver/data_router.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"

namespace autonomy {
namespace driver {

DataRouter::DataRouter(::autolink::Node* node, DriverEngine* driver_engine)
    : node_(node), driver_engine_(driver_engine) {
    if (node_ == nullptr) {
        AERROR << "DataRouter: node is null";
    }
    if (driver_engine_ == nullptr) {
        AERROR << "DataRouter: driver_engine is null";
    }
}

DataRouter::~DataRouter() {
    Stop();
}

bool DataRouter::Initialize(const proto::DriverOptions& options) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_) {
        AWARN << "DataRouter already initialized";
        return true;
    }

    if (node_ == nullptr || driver_engine_ == nullptr) {
        AERROR << "DataRouter: node or driver_engine is null, cannot initialize";
        return false;
    }

    // 保存默认转发目标
    default_forward_targets_.clear();
    for (const auto& target : options.default_forward_targets()) {
        default_forward_targets_.push_back(target);
    }

    // 为每个传感器检测并设置数据源类型
    // 收集所有传感器ID
    std::vector<std::string> sensor_ids;

    // 从 lidars 收集
    for (const auto& lidar : options.lidars()) {
        if (lidar.enabled()) {
            sensor_ids.push_back(lidar.sensor_id());
        }
    }

    // 从 imus 收集
    for (const auto& imu : options.imus()) {
        if (imu.enabled()) {
            sensor_ids.push_back(imu.sensor_id());
        }
    }

    // 从 cameras 收集
    for (const auto& camera : options.cameras()) {
        if (camera.enabled()) {
            sensor_ids.push_back(camera.sensor_id());
        }
    }

    // 从 ranges 收集
    for (const auto& range : options.ranges()) {
        if (range.enabled()) {
            sensor_ids.push_back(range.sensor_id());
        }
    }

    // 从 gps_sensors 收集
    for (const auto& gps : options.gps_sensors()) {
        if (gps.enabled()) {
            sensor_ids.push_back(gps.sensor_id());
        }
    }

    // 为每个传感器检测数据源类型
    for (const auto& sensor_id : sensor_ids) {
        DataSource source = DetectDataSource(sensor_id);
        data_sources_[sensor_id] = source;
        AINFO << "DataRouter: sensor " << sensor_id
              << " data source: " << (source == DataSource::ROS2 ? "ROS2" : "DRIVER");
    }

    initialized_ = true;
    AINFO << "DataRouter initialized with " << sensor_ids.size() << " sensors";
    return true;
}

void DataRouter::Start() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        AERROR << "DataRouter not initialized. Call Initialize() first.";
        return;
    }

    if (started_) {
        AWARN << "DataRouter already started";
        return;
    }

    // 为使用内部驱动的传感器注册数据处理器
    for (const auto& pair : data_sources_) {
        if (pair.second == DataSource::DRIVER) {
            // 注册到驱动引擎，当驱动数据到达时自动转发
            auto handler = [this](const std::string& sensor_id, const std::shared_ptr<sensor::Data>& data) {
                this->ForwardToTargets(sensor_id, data);
            };
            driver_engine_->RegisterSensorHandler(pair.first, handler);
        }
    }

    started_ = true;
    AINFO << "DataRouter started";
}

void DataRouter::Stop() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!started_) {
        return;
    }

    // 取消注册所有驱动引擎的处理器
    for (const auto& pair : data_sources_) {
        if (pair.second == DataSource::DRIVER) {
            driver_engine_->UnregisterSensorHandler(pair.first);
        }
    }

    // 清理所有发布器
    sensor_publishers_.clear();

    started_ = false;
    AINFO << "DataRouter stopped";
}

void DataRouter::SetDataSource(const std::string& sensor_id, DataSource source) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_sources_[sensor_id] = source;
    AINFO << "DataRouter: set sensor " << sensor_id << " data source to "
          << (source == DataSource::ROS2 ? "ROS2" : "DRIVER");
}

DataRouter::DataSource DataRouter::GetDataSource(const std::string& sensor_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = data_sources_.find(sensor_id);
    if (it != data_sources_.end()) {
        return it->second;
    }
    // 默认返回 DRIVER（如果没有明确设置，假设使用内部驱动）
    return DataSource::DRIVER;
}

void DataRouter::ForwardFromRos2(const std::string& sensor_id, const std::shared_ptr<sensor::Data>& data) {
    if (data == nullptr) {
        AERROR << "DataRouter: received null data from ROS2 for sensor: " << sensor_id;
        return;
    }

    // 检查数据源类型
    DataSource source = GetDataSource(sensor_id);
    if (source != DataSource::ROS2) {
        AWARN << "DataRouter: sensor " << sensor_id << " is not configured for ROS2, ignoring ROS2 data";
        return;
    }

    // 转发到目标
    ForwardToTargets(sensor_id, data);
}

void DataRouter::ReadAndSendFromDriver(const std::string& sensor_id) {
    // 检查数据源类型
    DataSource source = GetDataSource(sensor_id);
    if (source != DataSource::DRIVER) {
        AWARN << "DataRouter: sensor " << sensor_id << " is not configured for DRIVER, ignoring driver read request";
        return;
    }

    // 注意：实际的驱动数据读取由 DriverEngine 通过回调自动触发
    // 这个方法主要用于手动触发读取（如果需要的话）
    // 目前 DriverEngine 已经通过 RegisterSensorHandler 自动处理
    ADEBUG << "DataRouter: driver data for sensor " << sensor_id << " will be handled by DriverEngine callbacks";
}

void DataRouter::RegisterTargetHandler(
    const std::string& target, std::function<void(const std::string&, const std::shared_ptr<sensor::Data>&)> handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_handlers_[target] = handler;
    AINFO << "DataRouter: registered target handler for " << target;
}

void DataRouter::UnregisterTargetHandler(const std::string& target) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = target_handlers_.find(target);
    if (it != target_handlers_.end()) {
        target_handlers_.erase(it);
        AINFO << "DataRouter: unregistered target handler for " << target;
    }
}

void DataRouter::ForwardToTargets(const std::string& sensor_id, const std::shared_ptr<sensor::Data>& data,
                                  const std::vector<std::string>& targets) {
    if (data == nullptr) {
        AERROR << "DataRouter: cannot forward null data for sensor: " << sensor_id;
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // 确定要转发的目标列表
    std::vector<std::string> forward_targets;
    if (!targets.empty()) {
        forward_targets = targets;
    } else {
        // 使用默认目标
        forward_targets = default_forward_targets_;
    }

    // 转发到每个目标
    for (const auto& target : forward_targets) {
        auto it = target_handlers_.find(target);
        if (it != target_handlers_.end()) {
            try {
                it->second(sensor_id, data);
                ADEBUG << "DataRouter: forwarded data from sensor " << sensor_id << " to target " << target;
            } catch (const std::exception& e) {
                AERROR << "DataRouter: error forwarding data to target " << target << ": " << e.what();
            }
        } else {
            AWARN << "DataRouter: no handler registered for target " << target;
        }
    }
}

DataRouter::DataSource DataRouter::DetectDataSource(const std::string& sensor_id) const {
    // 检测策略：
    // 1. 如果传感器已经在驱动引擎中注册，则使用 DRIVER
    // 2. 否则，假设使用 ROS2（需要外部通过 ROS2 接口提供数据）

    if (driver_engine_ != nullptr && driver_engine_->IsSensorRegistered(sensor_id)) {
        return DataSource::DRIVER;
    }

    // 默认假设使用 ROS2（如果驱动引擎中没有注册，则数据应该来自 ROS2）
    // 注意：这个检测可能不够准确，建议通过配置明确指定数据源类型
    return DataSource::ROS2;
}

bool DataRouter::PublishLaserScan(const std::string& topic,
                                  const std::shared_ptr<commsgs::sensor_msgs::LaserScan>& message) {
    if (message == nullptr) {
        AERROR << "DataRouter: cannot publish null LaserScan message";
        return false;
    }

    if (node_ == nullptr) {
        AERROR << "DataRouter: node is null, cannot publish";
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // 获取或创建发布器
    auto it = sensor_publishers_.find(topic);
    if (it == sensor_publishers_.end() || it->second.sensor_type != "laser_scan") {
        auto writer = node_->CreateWriter<commsgs::sensor_msgs::LaserScan>(topic);
        if (writer == nullptr) {
            AERROR << "DataRouter: failed to create LaserScan writer for topic: " << topic;
            return false;
        }
        SensorPublisher publisher;
        publisher.topic = topic;
        publisher.sensor_type = "laser_scan";
        publisher.writer = std::static_pointer_cast<void>(writer);
        sensor_publishers_[topic] = publisher;
    }

    auto writer =
        std::static_pointer_cast<::autolink::Writer<commsgs::sensor_msgs::LaserScan>>(sensor_publishers_[topic].writer);
    return writer->Write(message);
}

bool DataRouter::PublishPointCloud2(const std::string& topic,
                                    const std::shared_ptr<commsgs::sensor_msgs::PointCloud2>& message) {
    if (message == nullptr) {
        AERROR << "DataRouter: cannot publish null PointCloud2 message";
        return false;
    }

    if (node_ == nullptr) {
        AERROR << "DataRouter: node is null, cannot publish";
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    auto it = sensor_publishers_.find(topic);
    if (it == sensor_publishers_.end() || it->second.sensor_type != "point_cloud2") {
        auto writer = node_->CreateWriter<commsgs::sensor_msgs::PointCloud2>(topic);
        if (writer == nullptr) {
            AERROR << "DataRouter: failed to create PointCloud2 writer for topic: " << topic;
            return false;
        }
        SensorPublisher publisher;
        publisher.topic = topic;
        publisher.sensor_type = "point_cloud2";
        publisher.writer = std::static_pointer_cast<void>(writer);
        sensor_publishers_[topic] = publisher;
    }

    auto writer = std::static_pointer_cast<::autolink::Writer<commsgs::sensor_msgs::PointCloud2>>(
        sensor_publishers_[topic].writer);
    return writer->Write(message);
}

bool DataRouter::PublishPointCloud(const std::string& topic,
                                   const std::shared_ptr<commsgs::sensor_msgs::PointCloud>& message) {
    if (message == nullptr) {
        AERROR << "DataRouter: cannot publish null PointCloud message";
        return false;
    }

    if (node_ == nullptr) {
        AERROR << "DataRouter: node is null, cannot publish";
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    auto it = sensor_publishers_.find(topic);
    if (it == sensor_publishers_.end() || it->second.sensor_type != "point_cloud") {
        auto writer = node_->CreateWriter<commsgs::sensor_msgs::PointCloud>(topic);
        if (writer == nullptr) {
            AERROR << "DataRouter: failed to create PointCloud writer for topic: " << topic;
            return false;
        }
        SensorPublisher publisher;
        publisher.topic = topic;
        publisher.sensor_type = "point_cloud";
        publisher.writer = std::static_pointer_cast<void>(writer);
        sensor_publishers_[topic] = publisher;
    }

    auto writer = std::static_pointer_cast<::autolink::Writer<commsgs::sensor_msgs::PointCloud>>(
        sensor_publishers_[topic].writer);
    return writer->Write(message);
}

bool DataRouter::PublishImu(const std::string& topic, const std::shared_ptr<commsgs::sensor_msgs::Imu>& message) {
    if (message == nullptr) {
        AERROR << "DataRouter: cannot publish null Imu message";
        return false;
    }

    if (node_ == nullptr) {
        AERROR << "DataRouter: node is null, cannot publish";
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    auto it = sensor_publishers_.find(topic);
    if (it == sensor_publishers_.end() || it->second.sensor_type != "imu") {
        auto writer = node_->CreateWriter<commsgs::sensor_msgs::Imu>(topic);
        if (writer == nullptr) {
            AERROR << "DataRouter: failed to create Imu writer for topic: " << topic;
            return false;
        }
        SensorPublisher publisher;
        publisher.topic = topic;
        publisher.sensor_type = "imu";
        publisher.writer = std::static_pointer_cast<void>(writer);
        sensor_publishers_[topic] = publisher;
    }

    auto writer =
        std::static_pointer_cast<::autolink::Writer<commsgs::sensor_msgs::Imu>>(sensor_publishers_[topic].writer);
    return writer->Write(message);
}

bool DataRouter::PublishOdometry(const std::string& topic,
                                 const std::shared_ptr<commsgs::planning_msgs::Odometry>& message) {
    if (message == nullptr) {
        AERROR << "DataRouter: cannot publish null Odometry message";
        return false;
    }

    if (node_ == nullptr) {
        AERROR << "DataRouter: node is null, cannot publish";
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    auto it = sensor_publishers_.find(topic);
    if (it == sensor_publishers_.end() || it->second.sensor_type != "odometry") {
        auto writer = node_->CreateWriter<commsgs::planning_msgs::Odometry>(topic);
        if (writer == nullptr) {
            AERROR << "DataRouter: failed to create Odometry writer for topic: " << topic;
            return false;
        }
        SensorPublisher publisher;
        publisher.topic = topic;
        publisher.sensor_type = "odometry";
        publisher.writer = std::static_pointer_cast<void>(writer);
        sensor_publishers_[topic] = publisher;
    }

    auto writer = std::static_pointer_cast<::autolink::Writer<commsgs::planning_msgs::Odometry>>(
        sensor_publishers_[topic].writer);
    return writer->Write(message);
}

bool DataRouter::PublishImage(const std::string& topic, const std::shared_ptr<commsgs::sensor_msgs::Image>& message) {
    if (message == nullptr) {
        AERROR << "DataRouter: cannot publish null Image message";
        return false;
    }

    if (node_ == nullptr) {
        AERROR << "DataRouter: node is null, cannot publish";
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    auto it = sensor_publishers_.find(topic);
    if (it == sensor_publishers_.end() || it->second.sensor_type != "image") {
        auto writer = node_->CreateWriter<commsgs::sensor_msgs::Image>(topic);
        if (writer == nullptr) {
            AERROR << "DataRouter: failed to create Image writer for topic: " << topic;
            return false;
        }
        SensorPublisher publisher;
        publisher.topic = topic;
        publisher.sensor_type = "image";
        publisher.writer = std::static_pointer_cast<void>(writer);
        sensor_publishers_[topic] = publisher;
    }

    auto writer =
        std::static_pointer_cast<::autolink::Writer<commsgs::sensor_msgs::Image>>(sensor_publishers_[topic].writer);
    return writer->Write(message);
}

bool DataRouter::PublishRange(const std::string& topic, const std::shared_ptr<commsgs::sensor_msgs::Range>& message) {
    if (message == nullptr) {
        AERROR << "DataRouter: cannot publish null Range message";
        return false;
    }

    if (node_ == nullptr) {
        AERROR << "DataRouter: node is null, cannot publish";
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    auto it = sensor_publishers_.find(topic);
    if (it == sensor_publishers_.end() || it->second.sensor_type != "range") {
        auto writer = node_->CreateWriter<commsgs::sensor_msgs::Range>(topic);
        if (writer == nullptr) {
            AERROR << "DataRouter: failed to create Range writer for topic: " << topic;
            return false;
        }
        SensorPublisher publisher;
        publisher.topic = topic;
        publisher.sensor_type = "range";
        publisher.writer = std::static_pointer_cast<void>(writer);
        sensor_publishers_[topic] = publisher;
    }

    auto writer =
        std::static_pointer_cast<::autolink::Writer<commsgs::sensor_msgs::Range>>(sensor_publishers_[topic].writer);
    return writer->Write(message);
}

}  // namespace driver
}  // namespace autonomy

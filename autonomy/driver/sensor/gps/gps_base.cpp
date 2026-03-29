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

#include "autonomy/driver/sensor/gps/gps_base.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "autolink/common/log.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace driver {
namespace sensor {
namespace gps {

using Time = autonomy::common::Time;

// GPS传感器数据包装类，继承 sensor::Data
class GpsSensorData : public autonomy::sensor::Data
{
public:
    GpsSensorData(const std::string& sensor_id,
                  const std::shared_ptr<NavSatFix>& msg)
        : autonomy::sensor::Data(sensor_id), message_(msg) {}

    Time GetTime() const override {
        // 将 builtin_interfaces::Time 转换为 common::Time
        int64_t total_ns =
            static_cast<int64_t>(message_->header.stamp.sec) * 1000000000LL +
            static_cast<int64_t>(message_->header.stamp.nanosec);
        return autonomy::common::FromUniversal(total_ns /
                                               100);  // 转换为 100ns ticks
    }

    void AddToCostmap(map::common::MapInterface* costmap_builder) override {
        (void)costmap_builder;
        // GPS数据通常不直接添加到 costmap
    }

    const std::shared_ptr<NavSatFix>& GetMessage() const {
        return message_;
    }

private:
    std::shared_ptr<NavSatFix> message_;
};

GpsBase::GpsBase() : stop_thread_(false) {}

GpsBase::~GpsBase() {
    Stop();
    Cleanup();
}

bool GpsBase::Configure(const std::string& name,
                        const proto::DriverOptions& options) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (configured_) {
        AWARN << "GpsBase already configured: " << name_;
        return true;
    }

    name_ = name;

    // 从 DriverOptions 中提取GPS配置
    gps_configs_.clear();
    for (const auto& gps_option : options.gps_sensors()) {
        if (gps_option.enabled()) {
            gps_configs_[gps_option.sensor_id()] = gps_option;
            AINFO << "Configured GPS sensor: " << gps_option.sensor_id()
                  << " (hardware: " << GetHardwareModel() << ")";
        }
    }

    if (gps_configs_.empty()) {
        AWARN << "No enabled GPS sensors found in configuration";
        return false;
    }

    configured_ = true;
    AINFO << "GpsBase configured: " << name_ << " with " << gps_configs_.size()
          << " GPS sensors";
    return true;
}

bool GpsBase::Initialize() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!configured_) {
        AERROR << "GpsBase not configured. Call Configure() first.";
        return false;
    }

    if (initialized_) {
        AWARN << "GpsBase already initialized: " << name_;
        return true;
    }

    // 检查硬件连接
    if (!IsConnected()) {
        AERROR << "GPS hardware not connected: " << GetHardwareModel();
        return false;
    }

    // 初始化每个传感器
    for (const auto& pair : gps_configs_) {
        const auto& config = pair.second;
        sampling_rate_ = config.sampling_rate();
        last_sample_time_[pair.first] = commsgs::builtin_interfaces::Time{0, 0};
        AINFO << "Initialized GPS sensor: " << pair.first
              << " (sampling_rate: " << sampling_rate_ << " Hz)";
    }

    initialized_ = true;
    AINFO << "GpsBase initialized: " << name_;
    return true;
}

void GpsBase::Start() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        AERROR << "GpsBase not initialized. Call Initialize() first.";
        return;
    }

    if (started_) {
        AWARN << "GpsBase already started: " << name_;
        return;
    }

    stop_thread_ = false;
    data_reading_thread_ = std::thread(&GpsBase::DataReadingThread, this);

    started_ = true;
    AINFO << "GpsBase started: " << name_;
}

void GpsBase::Stop() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!started_) {
        return;
    }

    stop_thread_ = true;

    if (data_reading_thread_.joinable()) {
        data_reading_thread_.join();
    }

    started_ = false;
    AINFO << "GpsBase stopped: " << name_;
}

void GpsBase::Cleanup() {
    std::lock_guard<std::mutex> lock(mutex_);

    Stop();

    handlers_.clear();
    gps_configs_.clear();
    last_sample_time_.clear();

    configured_ = false;
    initialized_ = false;
    AINFO << "GpsBase cleaned up: " << name_;
}

std::string GpsBase::GetName() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return name_;
}

std::vector<std::string> GpsBase::GetSensorIds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> sensor_ids;
    for (const auto& pair : gps_configs_) {
        sensor_ids.push_back(pair.first);
    }
    return sensor_ids;
}

bool GpsBase::IsSensorRegistered(const std::string& sensor_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return gps_configs_.find(sensor_id) != gps_configs_.end();
}

bool GpsBase::RegisterSensorHandler(
    const std::string& sensor_id,
    std::function<void(const std::string&,
                       const std::shared_ptr<autonomy::sensor::Data>&)>
        handler) {
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
    AINFO << "Registered sensor handler for GPS: " << sensor_id;
    return true;
}

void GpsBase::UnregisterSensorHandler(const std::string& sensor_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = handlers_.find(sensor_id);
    if (it != handlers_.end()) {
        handlers_.erase(it);
        AINFO << "Unregistered sensor handler for GPS: " << sensor_id;
    }
}

void GpsBase::ProcessGpsData(const std::string& sensor_id,
                             const std::shared_ptr<NavSatFix>& gps_msg) {
    if (gps_msg == nullptr) {
        AERROR << "Received null GPS message for sensor: " << sensor_id;
        return;
    }

    // 获取配置并检查精度阈值
    std::lock_guard<std::mutex> lock(mutex_);
    auto config_it = gps_configs_.find(sensor_id);
    if (config_it != gps_configs_.end()) {
        const auto& config = config_it->second;

        // 检查位置精度（如果协方差矩阵可用）
        if (!gps_msg->position_covariance.empty() &&
            gps_msg->position_covariance.size() >= 9) {
            // 计算位置精度（使用协方差矩阵的对角线元素）
            double position_variance =
                std::max(gps_msg->position_covariance[0],  // xx
                         gps_msg->position_covariance[4]   // yy
                );
            double position_accuracy = std::sqrt(position_variance);

            if (position_accuracy > config.min_position_accuracy()) {
                ADEBUG << "GPS position accuracy " << position_accuracy
                       << " exceeds threshold "
                       << config.min_position_accuracy()
                       << " for sensor: " << sensor_id;
                return;  // 过滤低精度数据
            }
        }

        // 检查高度精度（如果协方差矩阵可用）
        if (!gps_msg->position_covariance.empty() &&
            gps_msg->position_covariance.size() >= 9) {
            double altitude_variance = gps_msg->position_covariance[8];  // zz
            double altitude_accuracy = std::sqrt(altitude_variance);

            if (altitude_accuracy > config.min_altitude_accuracy()) {
                ADEBUG << "GPS altitude accuracy " << altitude_accuracy
                       << " exceeds threshold "
                       << config.min_altitude_accuracy()
                       << " for sensor: " << sensor_id;
                return;  // 过滤低精度数据
            }
        }
    }

    // 采样率控制
    if (sampling_rate_ > 0.0) {
        commsgs::builtin_interfaces::Time current_time = gps_msg->header.stamp;
        auto it = last_sample_time_.find(sensor_id);
        if (it != last_sample_time_.end()) {
            // 计算时间差（秒）
            int64_t time_diff_ns = (static_cast<int64_t>(current_time.sec) -
                                    static_cast<int64_t>(it->second.sec)) *
                                       1000000000LL +
                                   (static_cast<int64_t>(current_time.nanosec) -
                                    static_cast<int64_t>(it->second.nanosec));
            double time_diff = static_cast<double>(time_diff_ns) / 1e9;
            double min_interval = 1.0 / sampling_rate_;
            if (time_diff < min_interval) {
                // 跳过此数据（采样率限制）
                return;
            }
        }
        last_sample_time_[sensor_id] = current_time;
    }

    // 转换为 sensor::Data 并转发
    auto sensor_data = std::make_shared<GpsSensorData>(sensor_id, gps_msg);

    auto handler_it = handlers_.find(sensor_id);
    if (handler_it != handlers_.end()) {
        handler_it->second(sensor_id, sensor_data);
    } else {
        ADEBUG << "No handler registered for GPS sensor: " << sensor_id;
    }
}

void GpsBase::DataReadingThread() {
    AINFO << "GPS data reading thread started for: " << name_;

    // 计算读取间隔（如果采样率 > 0）
    std::chrono::milliseconds read_interval(1000);  // 默认 1Hz
    if (sampling_rate_ > 0.0) {
        read_interval = std::chrono::milliseconds(
            static_cast<int>(1000.0 / sampling_rate_));
    }

    while (!stop_thread_) {
        std::vector<std::string> sensor_ids;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& pair : gps_configs_) {
                sensor_ids.push_back(pair.first);
            }
        }

        // 读取每个传感器的数据
        for (const auto& sensor_id : sensor_ids) {
            auto gps_msg = ReadGpsData(sensor_id);
            if (gps_msg != nullptr) {
                ProcessGpsData(sensor_id, gps_msg);
            }
        }

        // 等待下一次读取
        std::this_thread::sleep_for(read_interval);
    }

    AINFO << "GPS data reading thread stopped for: " << name_;
}

}  // namespace gps
}  // namespace sensor
}  // namespace driver
}  // namespace autonomy

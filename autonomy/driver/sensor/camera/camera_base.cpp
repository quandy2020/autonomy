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

#include "autonomy/driver/sensor/camera/camera_base.hpp"

#include <chrono>
#include <thread>

#include "autolink/common/log.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace driver {
namespace sensor {
namespace camera {

using Time = autonomy::common::Time;

// 相机数据包装类，继承 sensor::Data
class CameraSensorData : public autonomy::sensor::Data
{
public:
    CameraSensorData(const std::string& sensor_id, const std::shared_ptr<commsgs::sensor_msgs::Image>& msg)
        : autonomy::sensor::Data(sensor_id), message_(msg) {}

    Time GetTime() const override {
        // 将 builtin_interfaces::Time 转换为 common::Time
        int64_t total_ns = static_cast<int64_t>(message_->header.stamp.sec) * 1000000000LL +
                           static_cast<int64_t>(message_->header.stamp.nanosec);
        return autonomy::common::FromUniversal(total_ns / 100);  // 转换为 100ns ticks
    }

    void AddToCostmap(map::common::MapInterface* costmap_builder) override {
        (void)costmap_builder;
        // 相机数据通常不直接添加到 costmap
    }

    const std::shared_ptr<commsgs::sensor_msgs::Image>& GetMessage() const {
        return message_;
    }

private:
    std::shared_ptr<commsgs::sensor_msgs::Image> message_;
};

CameraBase::CameraBase() : stop_thread_(false) {}

CameraBase::~CameraBase() {
    Stop();
    Cleanup();
}

bool CameraBase::Configure(const std::string& name, const proto::DriverOptions& options) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (configured_) {
        AWARN << "CameraBase already configured: " << name_;
        return true;
    }

    name_ = name;

    // 从 DriverOptions 中提取相机配置
    camera_configs_.clear();
    for (const auto& camera_option : options.cameras()) {
        if (camera_option.enabled()) {
            camera_configs_[camera_option.sensor_id()] = camera_option;
            AINFO << "Configured camera sensor: " << camera_option.sensor_id() << " (hardware: " << GetHardwareModel()
                  << ")";
        }
    }

    if (camera_configs_.empty()) {
        AWARN << "No enabled camera sensors found in configuration";
        return false;
    }

    configured_ = true;
    AINFO << "CameraBase configured: " << name_ << " with " << camera_configs_.size() << " camera sensors";
    return true;
}

bool CameraBase::Initialize() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!configured_) {
        AERROR << "CameraBase not configured. Call Configure() first.";
        return false;
    }

    if (initialized_) {
        AWARN << "CameraBase already initialized: " << name_;
        return true;
    }

    // 检查硬件连接
    if (!IsConnected()) {
        AERROR << "Camera hardware not connected: " << GetHardwareModel();
        return false;
    }

    // 初始化每个传感器
    for (const auto& pair : camera_configs_) {
        const auto& config = pair.second;
        sampling_rate_ = config.sampling_rate();
        last_sample_time_[pair.first] = commsgs::builtin_interfaces::Time{0, 0};
        AINFO << "Initialized camera sensor: " << pair.first << " (sampling_rate: " << sampling_rate_ << " Hz)";
    }

    initialized_ = true;
    AINFO << "CameraBase initialized: " << name_;
    return true;
}

void CameraBase::Start() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_) {
        AERROR << "CameraBase not initialized. Call Initialize() first.";
        return;
    }

    if (started_) {
        AWARN << "CameraBase already started: " << name_;
        return;
    }

    stop_thread_ = false;
    data_reading_thread_ = std::thread(&CameraBase::DataReadingThread, this);

    started_ = true;
    AINFO << "CameraBase started: " << name_;
}

void CameraBase::Stop() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!started_) {
        return;
    }

    stop_thread_ = true;

    if (data_reading_thread_.joinable()) {
        data_reading_thread_.join();
    }

    started_ = false;
    AINFO << "CameraBase stopped: " << name_;
}

void CameraBase::Cleanup() {
    std::lock_guard<std::mutex> lock(mutex_);

    Stop();

    handlers_.clear();
    camera_configs_.clear();
    last_sample_time_.clear();

    configured_ = false;
    initialized_ = false;
    AINFO << "CameraBase cleaned up: " << name_;
}

std::string CameraBase::GetName() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return name_;
}

std::vector<std::string> CameraBase::GetSensorIds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> sensor_ids;
    for (const auto& pair : camera_configs_) {
        sensor_ids.push_back(pair.first);
    }
    return sensor_ids;
}

bool CameraBase::IsSensorRegistered(const std::string& sensor_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return camera_configs_.find(sensor_id) != camera_configs_.end();
}

bool CameraBase::RegisterSensorHandler(
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
    AINFO << "Registered sensor handler for camera: " << sensor_id;
    return true;
}

void CameraBase::UnregisterSensorHandler(const std::string& sensor_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = handlers_.find(sensor_id);
    if (it != handlers_.end()) {
        handlers_.erase(it);
        AINFO << "Unregistered sensor handler for camera: " << sensor_id;
    }
}

void CameraBase::ProcessImageData(const std::string& sensor_id,
                                  const std::shared_ptr<commsgs::sensor_msgs::Image>& image_msg) {
    if (image_msg == nullptr) {
        AERROR << "Received null image message for sensor: " << sensor_id;
        return;
    }

    // 采样率控制
    if (sampling_rate_ > 0.0) {
        commsgs::builtin_interfaces::Time current_time = image_msg->header.stamp;
        auto it = last_sample_time_.find(sensor_id);
        if (it != last_sample_time_.end()) {
            // 计算时间差（秒）
            int64_t time_diff_ns =
                (static_cast<int64_t>(current_time.sec) - static_cast<int64_t>(it->second.sec)) * 1000000000LL +
                (static_cast<int64_t>(current_time.nanosec) - static_cast<int64_t>(it->second.nanosec));
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
    auto sensor_data = std::make_shared<CameraSensorData>(sensor_id, image_msg);

    std::lock_guard<std::mutex> lock(mutex_);
    auto handler_it = handlers_.find(sensor_id);
    if (handler_it != handlers_.end()) {
        handler_it->second(sensor_id, sensor_data);
    } else {
        ADEBUG << "No handler registered for camera sensor: " << sensor_id;
    }
}

void CameraBase::DataReadingThread() {
    AINFO << "Camera data reading thread started for: " << name_;

    // 计算读取间隔（如果采样率 > 0）
    std::chrono::milliseconds read_interval(33);  // 默认 30Hz
    if (sampling_rate_ > 0.0) {
        read_interval = std::chrono::milliseconds(static_cast<int>(1000.0 / sampling_rate_));
    }

    while (!stop_thread_) {
        std::vector<std::string> sensor_ids;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& pair : camera_configs_) {
                sensor_ids.push_back(pair.first);
            }
        }

        // 读取每个传感器的数据
        for (const auto& sensor_id : sensor_ids) {
            auto image_msg = ReadImageData(sensor_id);
            if (image_msg != nullptr) {
                ProcessImageData(sensor_id, image_msg);
            }

            // 如果是 RGBD 相机，也读取深度数据
            {
                std::lock_guard<std::mutex> lock(mutex_);
                auto config_it = camera_configs_.find(sensor_id);
                if (config_it != camera_configs_.end()) {
                    const auto& config = config_it->second;
                    if (config.type() == proto::CameraType::RGBD) {
                        auto depth_msg = ReadDepthData(sensor_id);
                        if (depth_msg != nullptr) {
                            // 使用 depth_topic 中定义的 sensor_id，如果存在
                            std::string depth_sensor_id =
                                config.depth_topic().empty() ? (sensor_id + "_depth") : config.depth_topic();
                            ProcessImageData(depth_sensor_id, depth_msg);
                        }
                    }
                }
            }
        }

        // 等待下一次读取
        std::this_thread::sleep_for(read_interval);
    }

    AINFO << "Camera data reading thread stopped for: " << name_;
}

}  // namespace camera
}  // namespace sensor
}  // namespace driver
}  // namespace autonomy

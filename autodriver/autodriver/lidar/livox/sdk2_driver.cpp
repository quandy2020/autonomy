/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autodriver/lidar/livox/sdk2_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <sstream>
#include <utility>

#include "autodriver/lidar/livox/convert.hpp"
#include "autodriver/lidar/livox/model.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/time.hpp"

#ifdef AUTODRIVER_HAVE_LIVOX_SDK2
#include <livox_lidar_api.h>
#include <livox_lidar_def.h>
#endif

namespace autodriver {
namespace hardware {
namespace {

std::uint64_t IntervalFromHz(double hz) {
    if (hz <= 0.1) {
        hz = 10.0;
    }
    return static_cast<std::uint64_t>(1e9 / hz);
}

std::uint64_t ReadTimestampNs(const std::uint8_t* stamp8) {
    std::uint64_t t = 0;
    std::memcpy(&t, stamp8, sizeof(t));
    return t;
}

#ifdef AUTODRIVER_HAVE_LIVOX_SDK2

void WorkModeCb(livox_status, uint32_t, LivoxLidarAsyncControlResponse*,
                void*) {}

void LivoxSdk2PointCloudThunk(uint32_t handle, const uint8_t dev_type,
                              LivoxLidarEthernetPacket* data,
                              void* client) {
    auto* self = static_cast<LivoxSdk2Driver*>(client);
    if (self != nullptr) {
        self->OnPointCloud(handle, dev_type, data);
    }
}

void LivoxSdk2InfoChangeThunk(const uint32_t handle, const LivoxLidarInfo* info,
                              void* client) {
    auto* self = static_cast<LivoxSdk2Driver*>(client);
    if (self != nullptr) {
        self->OnInfoChange(handle, info);
    }
}

#endif  // AUTODRIVER_HAVE_LIVOX_SDK2

}  // namespace

LivoxSdk2Driver::LivoxSdk2Driver(SensorId id, DriverParams params)
    : id_(std::move(id)),
      params_(std::move(params)),
      assembler_(IntervalFromHz(10.0)) {
    model_ = GetString(params_, "model", "Mid-360");
    frame_id_ = GetString(params_, "frame_id", id_);
    config_path_ = GetString(params_, "config_path", "");
    host_ip_ = GetString(params_, "host_ip", "192.168.1.5");
    lidar_ip_ = GetString(params_, "lidar_ip", "192.168.1.12");
    publish_freq_hz_ = ParseDouble(params_, "publish_freq", 0.0);
    if (publish_freq_hz_ <= 0.0) {
        publish_freq_hz_ = ParseDouble(params_, "fps", 10.0);
    }
    if (publish_freq_hz_ <= 0.0) {
        publish_freq_hz_ = 10.0;
    }
    pcl_data_type_ = ParseInt(params_, "pcl_data_type", 1);
    assembler_.SetIntervalNs(IntervalFromHz(publish_freq_hz_));

    lidar::LidarBaseOptions options;
    options.source = lidar::ParseSourceType(
        GetString(params_, "source_type", "online"));
    options.cloud_channel = GetString(params_, "channel", "");
    options.publish_scan = false;
    InitBase(options);
}

LivoxSdk2Driver::~LivoxSdk2Driver() { Stop(); }

bool LivoxSdk2Driver::IsRunning() const { return running_.load(); }

void LivoxSdk2Driver::SetSampleCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

void LivoxSdk2Driver::WritePointCloud(std::shared_ptr<SensorSample> cloud) {
    if (callback_ && cloud) {
        callback_(cloud->Clone());
    }
}

bool LivoxSdk2Driver::EnsureConfigFile(std::string* path, std::string* err) {
    if (path == nullptr) {
        return false;
    }
    if (!config_path_.empty()) {
        *path = config_path_;
        return true;
    }
    const std::string key = lidar::livox::Sdk2JsonModelKey(model_);
    generated_config_path_ =
        "/tmp/autodriver_livox_" +
        std::to_string(std::hash<std::string>{}(id_)) + ".json";

    std::ostringstream oss;
    oss << "{\n"
        << "  \"lidar_summary_info\": { \"lidar_type\": 8 },\n"
        << "  \"" << key << "\": {\n"
        << "    \"lidar_net_info\": {\n"
        << "      \"cmd_data_port\": 56100,\n"
        << "      \"push_msg_port\": 56200,\n"
        << "      \"point_data_port\": 56300,\n"
        << "      \"imu_data_port\": 56400,\n"
        << "      \"log_data_port\": 56500\n"
        << "    },\n"
        << "    \"host_net_info\": {\n"
        << "      \"cmd_data_ip\": \"" << host_ip_ << "\",\n"
        << "      \"cmd_data_port\": 56101,\n"
        << "      \"push_msg_ip\": \"" << host_ip_ << "\",\n"
        << "      \"push_msg_port\": 56201,\n"
        << "      \"point_data_ip\": \"" << host_ip_ << "\",\n"
        << "      \"point_data_port\": 56301,\n"
        << "      \"imu_data_ip\": \"" << host_ip_ << "\",\n"
        << "      \"imu_data_port\": 56401,\n"
        << "      \"log_data_ip\": \"\",\n"
        << "      \"log_data_port\": 56501\n"
        << "    }\n"
        << "  },\n"
        << "  \"lidar_configs\": [{\n"
        << "    \"ip\": \"" << lidar_ip_ << "\",\n"
        << "    \"pcl_data_type\": " << pcl_data_type_ << ",\n"
        << "    \"pattern_mode\": 0,\n"
        << "    \"extrinsic_parameter\": {\n"
        << "      \"roll\": 0.0, \"pitch\": 0.0, \"yaw\": 0.0,\n"
        << "      \"x\": 0, \"y\": 0, \"z\": 0\n"
        << "    }\n"
        << "  }]\n"
        << "}\n";

    std::ofstream out(generated_config_path_);
    if (!out) {
        if (err != nullptr) {
            *err = "cannot write " + generated_config_path_;
        }
        return false;
    }
    out << oss.str();
    *path = generated_config_path_;
    AINFO << "Livox SDK2 generated config " << *path;
    return true;
}

bool LivoxSdk2Driver::InitSdk() {
#ifdef AUTODRIVER_HAVE_LIVOX_SDK2
    std::string path;
    std::string err;
    if (!EnsureConfigFile(&path, &err)) {
        AERROR << "Livox SDK2 config: " << err;
        return false;
    }
    if (!LivoxLidarSdkInit(path.c_str())) {
        AERROR << "LivoxLidarSdkInit failed (" << path << ")";
        LivoxLidarSdkUninit();
        return false;
    }
    SetLivoxLidarPointCloudCallBack(LivoxSdk2PointCloudThunk, this);
    SetLivoxLidarInfoChangeCallback(LivoxSdk2InfoChangeThunk, this);
    sdk_owned_ = true;
    return true;
#else
    AERROR << "Livox SDK2 not available; install via "
              "scripts/install_livox_sdk2.sh";
    return false;
#endif
}

void LivoxSdk2Driver::UninitSdk() {
#ifdef AUTODRIVER_HAVE_LIVOX_SDK2
    if (sdk_owned_) {
        LivoxLidarSdkUninit();
        sdk_owned_ = false;
    }
#endif
}

bool LivoxSdk2Driver::Start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return true;
    }
    if (!InitSdk()) {
        running_ = false;
        return false;
    }
    publisher_ = std::thread([this] { PublishLoop(); });
    AINFO << "Livox SDK2 driver started id=" << id_ << " model=" << model_;
    return true;
}

void LivoxSdk2Driver::Stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (publisher_.joinable()) {
        publisher_.join();
    }
    assembler_.Clear();
    UninitSdk();
}

void LivoxSdk2Driver::PublishLoop() {
    while (running_.load()) {
        std::vector<lidar::livox::PointXYZIT> frame;
        const std::uint64_t now_ns =
            static_cast<std::uint64_t>(autolink::Time::Now().ToNanosecond());
        if (assembler_.TryFlush(now_ns, &frame) && !frame.empty()) {
            PublishFrame(std::move(frame));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void LivoxSdk2Driver::PublishFrame(
    std::vector<lidar::livox::PointXYZIT> points) {
    auto cloud_msg =
        lidar::livox::PointsToPointCloud(points, frame_id_);
    auto sample = std::make_shared<LidarCloud>(
        id_, autolink::Time::Now(), std::move(cloud_msg));
    sample->frame_id = frame_id_;
    sample->channel = options().cloud_channel;
    WritePointCloud(sample);
}

void LivoxSdk2Driver::OnInfoChange(std::uint32_t handle, const void* info) {
#ifdef AUTODRIVER_HAVE_LIVOX_SDK2
    const auto* lidar_info = static_cast<const LivoxLidarInfo*>(info);
    if (lidar_info == nullptr) {
        return;
    }
    AINFO << "Livox SDK2 lidar online handle=" << handle
          << " sn=" << lidar_info->sn;
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCb, nullptr);
#else
    (void)handle;
    (void)info;
#endif
}

void LivoxSdk2Driver::OnPointCloud(std::uint32_t /*handle*/,
                                   std::uint8_t /*dev_type*/, void* data) {
#ifdef AUTODRIVER_HAVE_LIVOX_SDK2
    auto* packet = static_cast<LivoxLidarEthernetPacket*>(data);
    if (packet == nullptr || packet->dot_num == 0) {
        return;
    }
    const std::uint64_t base_ns = ReadTimestampNs(packet->timestamp);
    const std::uint64_t point_interval_ns =
        packet->dot_num == 0
            ? 0
            : static_cast<std::uint64_t>(packet->time_interval) * 100ULL /
                  packet->dot_num;

    std::vector<lidar::livox::PointXYZIT> points;
    points.reserve(packet->dot_num * 2);

    auto push_xyz = [&](float x, float y, float z, float intensity,
                        std::uint32_t i) {
        if (x == 0.f && y == 0.f && z == 0.f) {
            return;
        }
        lidar::livox::PointXYZIT p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = intensity;
        p.timestamp_ns =
            static_cast<double>(base_ns + i * point_interval_ns);
        points.push_back(p);
    };

    if (packet->data_type == kLivoxLidarCartesianCoordinateHighData) {
        auto* raw =
            reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(packet->data);
        for (std::uint32_t i = 0; i < packet->dot_num; ++i) {
            push_xyz(raw[i].x * 0.001f, raw[i].y * 0.001f, raw[i].z * 0.001f,
                     static_cast<float>(raw[i].reflectivity), i);
        }
    } else if (packet->data_type == kLivoxLidarCartesianCoordinateLowData) {
        auto* raw =
            reinterpret_cast<LivoxLidarCartesianLowRawPoint*>(packet->data);
        for (std::uint32_t i = 0; i < packet->dot_num; ++i) {
            push_xyz(raw[i].x * 0.01f, raw[i].y * 0.01f, raw[i].z * 0.01f,
                     static_cast<float>(raw[i].reflectivity), i);
        }
    } else if (packet->data_type == kLivoxLidarSphericalCoordinateData) {
        auto* raw = reinterpret_cast<LivoxLidarSpherPoint*>(packet->data);
        constexpr double kScale = 0.01 * M_PI / 180.0;
        for (std::uint32_t i = 0; i < packet->dot_num; ++i) {
            const double depth = raw[i].depth * 0.001;
            const double theta = raw[i].theta * kScale;
            const double phi = raw[i].phi * kScale;
            const float x =
                static_cast<float>(depth * std::sin(theta) * std::cos(phi));
            const float y =
                static_cast<float>(depth * std::sin(theta) * std::sin(phi));
            const float z = static_cast<float>(depth * std::cos(theta));
            push_xyz(x, y, z, static_cast<float>(raw[i].reflectivity), i);
        }
    } else if (packet->data_type == kLivoxLidarDoubleEchoData) {
        auto* raw =
            reinterpret_cast<LivoxLidarDoubleEchoRawPoint*>(packet->data);
        for (std::uint32_t i = 0; i < packet->dot_num; ++i) {
            push_xyz(raw[i].x1 * 0.001f, raw[i].y1 * 0.001f,
                     raw[i].z1 * 0.001f,
                     static_cast<float>(raw[i].reflectivity1), i);
            push_xyz(raw[i].x2 * 0.001f, raw[i].y2 * 0.001f,
                     raw[i].z2 * 0.001f,
                     static_cast<float>(raw[i].reflectivity2), i);
        }
    }

    if (!points.empty()) {
        assembler_.Append(std::move(points));
    }
#else
    (void)data;
#endif
}

std::shared_ptr<SensorDriver> CreateLivoxSdk2Driver(
    const SensorId& id, const DriverParams& params) {
#ifndef AUTODRIVER_HAVE_LIVOX_SDK2
    AERROR << "Livox SDK2 not linked; cannot create driver for " << id;
    return nullptr;
#else
    return std::make_shared<LivoxSdk2Driver>(id, params);
#endif
}

}  // namespace hardware
}  // namespace autodriver

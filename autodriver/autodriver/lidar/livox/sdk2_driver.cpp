/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

/**
 * @file sdk2_driver.cpp
 * @brief Livox-SDK2 driver (HAP / Mid-360 / Mid360s / Avia2) (implementation).
 */

#include "autodriver/lidar/livox/sdk2_driver.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <sstream>
#include <utility>
#include <vector>

#include "autodriver/lidar/livox/model.hpp"
#include "autolink/common/log.hpp"

#ifdef AUTODRIVER_HAVE_LIVOX_SDK2
#include <livox_lidar_api.h>
#include <livox_lidar_def.h>
#endif

namespace autodriver {
namespace hardware {
namespace {

#ifdef AUTODRIVER_HAVE_LIVOX_SDK2

std::uint64_t ReadTimestampNs(const std::uint8_t* stamp8) {
    std::uint64_t t = 0;
    std::memcpy(&t, stamp8, sizeof(t));
    return t;
}

void WorkModeCb(livox_status, uint32_t, LivoxLidarAsyncControlResponse*,
                void*) {}

void LivoxSdk2PointCloudThunk(uint32_t handle, const uint8_t dev_type,
                              LivoxLidarEthernetPacket* data, void* client) {
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
    : Base(std::move(id), std::move(params)) {
    config_path_ = GetString(this->params(), "config_path", "");
    host_ip_ = GetString(this->params(), "host_ip", "192.168.1.5");
    lidar_ip_ = GetString(this->params(), "lidar_ip", "192.168.1.12");
    pcl_data_type_ = ParseInt(this->params(), "pcl_data_type", 1);
}

bool LivoxSdk2Driver::EnsureConfigFile(std::string* path, std::string* err) {
    if (path == nullptr) {
        return false;
    }
    if (!config_path_.empty()) {
        *path = config_path_;
        return true;
    }
    const std::string key = lidar::livox::Sdk2JsonModelKey(model());
    generated_config_path_ =
        "/tmp/autodriver_livox_" +
        std::to_string(std::hash<std::string>{}(id())) + ".json";

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
    set_sdk_owned(true);
    return true;
#else
    AERROR << "Livox SDK2 not available; install via "
              "scripts/install_livox_sdk2.sh";
    return false;
#endif
}

void LivoxSdk2Driver::UninitSdk() {
#ifdef AUTODRIVER_HAVE_LIVOX_SDK2
    if (sdk_owned()) {
        LivoxLidarSdkUninit();
        set_sdk_owned(false);
    }
#endif
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

    AppendPoints(std::move(points));
#else
    (void)data;
#endif
}

SensorDriver* CreateLivoxSdk2Driver(const SensorId& id,
                                    const DriverParams& params) {
#ifndef AUTODRIVER_HAVE_LIVOX_SDK2
    (void)params;
    AERROR << "Livox SDK2 not linked; cannot create driver for " << id;
    return nullptr;
#else
    return new LivoxSdk2Driver(id, params);
#endif
}

}  // namespace hardware
}  // namespace autodriver

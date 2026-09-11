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
 * @file sdk1_driver.cpp
 * @brief Livox-SDK (v1) driver — Mid-40/70, Horizon, Avia, Tele (implementation).
 */

#include "autodriver/lidar/livox/sdk1_driver.hpp"

#include <cmath>
#include <cstring>
#include <mutex>
#include <utility>
#include <vector>

#include "autodriver/lidar/livox/common.hpp"
#include "autolink/common/log.hpp"

#ifdef AUTODRIVER_HAVE_LIVOX_SDK1
#include "livox_def.h"
#include "livox_sdk.h"
#endif

namespace autodriver {
namespace hardware {
namespace {

#ifdef AUTODRIVER_HAVE_LIVOX_SDK1

std::uint64_t ReadTimestampNs(const std::uint8_t* stamp8) {
    std::uint64_t t = 0;
    std::memcpy(&t, stamp8, sizeof(t));
    return t;
}

void OnSampleCb(livox_status, uint8_t, uint8_t, void*) {}

void LivoxSdk1DataThunk(uint8_t handle, LivoxEthPacket* data,
                        uint32_t data_num, void* client) {
    auto* self = static_cast<LivoxSdk1Driver*>(client);
    if (self != nullptr) {
        self->OnData(handle, data, data_num);
    }
}

std::mutex g_sdk1_mutex;
LivoxSdk1Driver* g_sdk1_active = nullptr;

void LivoxSdk1BroadcastGlobal(const BroadcastDeviceInfo* info) {
    std::lock_guard<std::mutex> lock(g_sdk1_mutex);
    if (g_sdk1_active != nullptr) {
        g_sdk1_active->OnBroadcast(info);
    }
}

void LivoxSdk1InfoChangeGlobal(const DeviceInfo* info, DeviceEvent type) {
    std::lock_guard<std::mutex> lock(g_sdk1_mutex);
    if (g_sdk1_active != nullptr) {
        g_sdk1_active->OnInfoChange(info, static_cast<std::uint8_t>(type));
    }
}

#endif  // AUTODRIVER_HAVE_LIVOX_SDK1

}  // namespace

LivoxSdk1Driver::LivoxSdk1Driver(SensorId id, DriverParams params)
    : Base(std::move(id), std::move(params)) {
    std::string codes = GetString(this->params(), "broadcast_code", "");
    if (codes.empty()) {
        codes = GetString(this->params(), "broadcast_codes", "");
    }
    if (codes.empty()) {
        codes = GetString(this->params(), "bd_code", "");
    }
    broadcast_codes_ = lidar::livox::ParseBroadcastCodes(codes);
}

bool LivoxSdk1Driver::InitSdk() {
#ifdef AUTODRIVER_HAVE_LIVOX_SDK1
    {
        std::lock_guard<std::mutex> lock(g_sdk1_mutex);
        if (g_sdk1_active != nullptr && g_sdk1_active != this) {
            AERROR << "Livox SDK1 already in use by another driver";
            return false;
        }
        g_sdk1_active = this;
    }
    if (!Init()) {
        AERROR << "Livox-SDK Init() failed";
        std::lock_guard<std::mutex> lock(g_sdk1_mutex);
        if (g_sdk1_active == this) {
            g_sdk1_active = nullptr;
        }
        return false;
    }
    SetBroadcastCallback(LivoxSdk1BroadcastGlobal);
    SetDeviceStateUpdateCallback(LivoxSdk1InfoChangeGlobal);
    if (!::Start()) {
        AERROR << "Livox-SDK Start() failed";
        Uninit();
        std::lock_guard<std::mutex> lock(g_sdk1_mutex);
        if (g_sdk1_active == this) {
            g_sdk1_active = nullptr;
        }
        return false;
    }
    set_sdk_owned(true);
    return true;
#else
    AERROR << "Livox SDK1 not available; install via "
              "scripts/install_livox_sdk.sh";
    return false;
#endif
}

void LivoxSdk1Driver::UninitSdk() {
#ifdef AUTODRIVER_HAVE_LIVOX_SDK1
    if (sdk_owned()) {
        Uninit();
        set_sdk_owned(false);
    }
    std::lock_guard<std::mutex> lock(g_sdk1_mutex);
    if (g_sdk1_active == this) {
        g_sdk1_active = nullptr;
    }
#endif
}

void LivoxSdk1Driver::OnBroadcast(const void* info) {
#ifdef AUTODRIVER_HAVE_LIVOX_SDK1
    const auto* binfo = static_cast<const BroadcastDeviceInfo*>(info);
    if (binfo == nullptr || binfo->dev_type == kDeviceTypeHub) {
        return;
    }
    if (!broadcast_codes_.empty() &&
        broadcast_codes_.count(binfo->broadcast_code) == 0) {
        return;
    }
    uint8_t handle = 0;
    if (AddLidarToConnect(binfo->broadcast_code, &handle) == kStatusSuccess) {
        SetDataCallback(handle, LivoxSdk1DataThunk, this);
        AINFO << "Livox SDK1 connecting " << binfo->broadcast_code
              << " handle=" << static_cast<int>(handle);
    }
#else
    (void)info;
#endif
}

void LivoxSdk1Driver::OnInfoChange(const void* info, std::uint8_t type) {
#ifdef AUTODRIVER_HAVE_LIVOX_SDK1
    const auto* dinfo = static_cast<const DeviceInfo*>(info);
    if (dinfo == nullptr) {
        return;
    }
    const auto event = static_cast<DeviceEvent>(type);
    if (event == kEventConnect || event == kEventStateChange) {
        if (dinfo->state == kLidarStateNormal) {
            LidarStartSampling(dinfo->handle, OnSampleCb, nullptr);
            AINFO << "Livox SDK1 sampling " << dinfo->broadcast_code;
        }
    }
#else
    (void)info;
    (void)type;
#endif
}

void LivoxSdk1Driver::OnData(std::uint8_t /*handle*/, void* data,
                             std::uint32_t data_num) {
#ifdef AUTODRIVER_HAVE_LIVOX_SDK1
    auto* packet = static_cast<LivoxEthPacket*>(data);
    if (packet == nullptr || data_num == 0) {
        return;
    }
    const std::uint64_t base_ns = ReadTimestampNs(packet->timestamp);
    std::vector<lidar::livox::PointXYZIT> points;
    points.reserve(data_num * 2);

    auto push_mm = [&](int32_t x, int32_t y, int32_t z, uint8_t refl,
                       std::uint32_t i) {
        if (x == 0 && y == 0 && z == 0) {
            return;
        }
        lidar::livox::PointXYZIT p;
        p.x = x * 0.001f;
        p.y = y * 0.001f;
        p.z = z * 0.001f;
        p.intensity = static_cast<float>(refl);
        p.timestamp_ns = static_cast<double>(base_ns + i);
        points.push_back(p);
    };

    if (packet->data_type == kCartesian) {
        auto* raw = reinterpret_cast<LivoxRawPoint*>(packet->data);
        for (std::uint32_t i = 0; i < data_num; ++i) {
            push_mm(raw[i].x, raw[i].y, raw[i].z, raw[i].reflectivity, i);
        }
    } else if (packet->data_type == kExtendCartesian) {
        auto* raw = reinterpret_cast<LivoxExtendRawPoint*>(packet->data);
        for (std::uint32_t i = 0; i < data_num; ++i) {
            push_mm(raw[i].x, raw[i].y, raw[i].z, raw[i].reflectivity, i);
        }
    } else if (packet->data_type == kDualExtendCartesian) {
        auto* raw = reinterpret_cast<LivoxDualExtendRawPoint*>(packet->data);
        for (std::uint32_t i = 0; i < data_num; ++i) {
            push_mm(raw[i].x1, raw[i].y1, raw[i].z1, raw[i].reflectivity1,
                    i);
            push_mm(raw[i].x2, raw[i].y2, raw[i].z2, raw[i].reflectivity2,
                    i);
        }
    } else if (packet->data_type == kSpherical ||
               packet->data_type == kExtendSpherical) {
        auto* raw = reinterpret_cast<LivoxSpherPoint*>(packet->data);
        constexpr double kScale = 0.01 * M_PI / 180.0;
        for (std::uint32_t i = 0; i < data_num; ++i) {
            const double depth = raw[i].depth * 0.001;
            const double theta = raw[i].theta * kScale;
            const double phi = raw[i].phi * kScale;
            lidar::livox::PointXYZIT p;
            p.x = static_cast<float>(depth * std::sin(theta) * std::cos(phi));
            p.y = static_cast<float>(depth * std::sin(theta) * std::sin(phi));
            p.z = static_cast<float>(depth * std::cos(theta));
            p.intensity = static_cast<float>(raw[i].reflectivity);
            p.timestamp_ns = static_cast<double>(base_ns + i);
            if (p.x != 0.f || p.y != 0.f || p.z != 0.f) {
                points.push_back(p);
            }
        }
    }

    AppendPoints(std::move(points));
#else
    (void)data;
    (void)data_num;
#endif
}

SensorDriver* CreateLivoxSdk1Driver(const SensorId& id,
                                    const DriverParams& params) {
#ifndef AUTODRIVER_HAVE_LIVOX_SDK1
    (void)params;
    AERROR << "Livox SDK1 not linked; cannot create driver for " << id;
    return nullptr;
#else
    return new LivoxSdk1Driver(id, params);
#endif
}

}  // namespace hardware
}  // namespace autodriver

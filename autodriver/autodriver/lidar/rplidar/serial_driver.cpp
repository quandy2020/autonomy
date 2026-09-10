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

#include "autodriver/lidar/rplidar/serial_driver.hpp"

#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <vector>

#include "autodriver/lidar/lidar_2d_backend_register.hpp"
#include "autodriver/lidar/rplidar/convert.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/time.hpp"

#ifdef AUTODRIVER_HAVE_RPLIDAR
#include <sl_lidar.h>
#include <sl_lidar_driver.h>
#endif

namespace autodriver {
namespace lidar {
namespace rplidar {
namespace {

int DefaultBaudForModel(const std::string& model) {
    std::string m = model;
    for (char& c : m) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    // A3 / A2M7 / A2M12 use 256000; A1 / A2M8 use 115200 (rplidar_ros launch).
    if (m.find("a3") != std::string::npos ||
        m.find("a2m7") != std::string::npos ||
        m.find("a2m12") != std::string::npos ||
        m.find("s1") != std::string::npos) {
        return 256000;
    }
    return 115200;
}

}  // namespace

RpLidarSerialDriver::RpLidarSerialDriver(SensorId id,
                                         hardware::DriverParams params)
    : id_(std::move(id)), params_(std::move(params)) {
    frame_id_ = hardware::GetString(params_, "frame_id", "laser");
}

RpLidarSerialDriver::~RpLidarSerialDriver() { Stop(); }

void RpLidarSerialDriver::SetSampleCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

#ifndef AUTODRIVER_HAVE_RPLIDAR

bool RpLidarSerialDriver::Start() {
    AERROR << "RPLidar driver built without SDK (AUTODRIVER_HAVE_RPLIDAR)";
    return false;
}

void RpLidarSerialDriver::Stop() {}

bool RpLidarSerialDriver::ConnectDevice() { return false; }
void RpLidarSerialDriver::DisconnectDevice() {}
bool RpLidarSerialDriver::StartMotorAndScan() { return false; }
void RpLidarSerialDriver::CaptureLoop() {}

std::shared_ptr<SensorDriver> CreateRpLidarDriver(
    const SensorId& /*id*/, const hardware::DriverParams& /*params*/) {
    AERROR << "RPLidar SDK not available; install via "
              "scripts/install_rplidar_sdk.sh and rebuild with "
              "AUTODRIVER_WITH_RPLIDAR";
    return nullptr;
}

#else  // AUTODRIVER_HAVE_RPLIDAR

using namespace sl;

bool RpLidarSerialDriver::ConnectDevice() {
    const std::string channel_type =
        hardware::GetString(params_, "channel_type", "serial");
    IChannel* channel = nullptr;
    if (channel_type == "tcp") {
        const std::string ip =
            hardware::GetString(params_, "tcp_ip", "192.168.0.7");
        const int port = hardware::ParseInt(params_, "tcp_port", 20108);
        channel = *createTcpChannel(ip, static_cast<int>(port));
    } else if (channel_type == "udp") {
        const std::string ip =
            hardware::GetString(params_, "udp_ip", "192.168.11.2");
        const int port = hardware::ParseInt(params_, "udp_port", 8089);
        channel = *createUdpChannel(ip, static_cast<int>(port));
    } else {
        std::string device = hardware::GetString(params_, "device");
        if (device.empty()) {
            device = hardware::GetString(params_, "port", "/dev/ttyUSB0");
        }
        const std::string model = hardware::GetString(params_, "model", "A1");
        const int baud = hardware::ParseInt(
            params_, "baud",
            hardware::ParseInt(params_, "baudrate",
                               DefaultBaudForModel(model)));
        channel = *createSerialPortChannel(device, baud);
        AINFO << "RPLidar serial " << device << " @" << baud
              << " model=" << model;
    }
    if (!channel) {
        AERROR << "RPLidar: failed to create channel";
        return false;
    }

    ILidarDriver* drv = *createLidarDriver();
    if (!drv) {
        delete channel;
        AERROR << "RPLidar: failed to create driver";
        return false;
    }
    const sl_result rc = drv->connect(channel);
    if (SL_IS_FAIL(rc)) {
        delete drv;
        delete channel;
        AERROR << "RPLidar: connect failed code=" << std::hex << rc;
        return false;
    }

    sl_lidar_response_device_info_t info{};
    if (SL_IS_FAIL(drv->getDeviceInfo(info))) {
        drv->disconnect();
        delete drv;
        delete channel;
        AERROR << "RPLidar: getDeviceInfo failed";
        return false;
    }
    AINFO << "RPLidar firmware " << (info.firmware_version >> 8) << "."
          << (info.firmware_version & 0xFF)
          << " hw=" << static_cast<int>(info.hardware_version);

    sl_lidar_response_device_health_t health{};
    if (SL_IS_OK(drv->getHealth(health))) {
        if (health.status == SL_LIDAR_STATUS_ERROR) {
            AERROR << "RPLidar health ERROR; try reboot";
            drv->disconnect();
            delete drv;
            delete channel;
            return false;
        }
    }

    if (hardware::ParseBool(params_, "initial_reset", false)) {
        drv->reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    channel_ = channel;
    driver_ = drv;
    return true;
}

void RpLidarSerialDriver::DisconnectDevice() {
    auto* drv = static_cast<ILidarDriver*>(driver_);
    auto* channel = static_cast<IChannel*>(channel_);
    if (drv) {
        drv->stop();
        drv->setMotorSpeed(0);
        drv->disconnect();
        delete drv;
    }
    delete channel;
    driver_ = nullptr;
    channel_ = nullptr;
}

bool RpLidarSerialDriver::StartMotorAndScan() {
    auto* drv = static_cast<ILidarDriver*>(driver_);
    if (!drv) {
        return false;
    }
    // A-series motor PWM ~10 Hz default (rplidar_ros).
    drv->setMotorSpeed(600);

    const float scan_frequency =
        static_cast<float>(hardware::ParseDouble(params_, "scan_frequency", 10.0));
    const std::string scan_mode = hardware::GetString(params_, "scan_mode");
    LidarScanMode current{};
    sl_result op = SL_RESULT_OK;

    if (scan_mode.empty()) {
        op = drv->startScan(false, true, 0, &current);
    } else {
        std::vector<LidarScanMode> modes;
        op = drv->getAllSupportedScanModes(modes);
        if (SL_IS_OK(op)) {
            sl_u16 selected = static_cast<sl_u16>(-1);
            for (const auto& m : modes) {
                if (scan_mode == m.scan_mode) {
                    selected = m.id;
                    break;
                }
            }
            if (selected == static_cast<sl_u16>(-1)) {
                AERROR << "RPLidar scan_mode not supported: " << scan_mode;
                return false;
            }
            op = drv->startScanExpress(false, selected, 0, &current);
        }
    }
    if (SL_IS_FAIL(op)) {
        AERROR << "RPLidar startScan failed: " << std::hex << op;
        return false;
    }

    max_distance_ = current.max_distance > 0.f ? current.max_distance : 12.f;
    const float points_per_circle =
        (1000.f * 1000.f) / current.us_per_sample / scan_frequency;
    angle_compensate_multiple_ = points_per_circle / 360.f + 1.f;
    if (angle_compensate_multiple_ < 1.f) {
        angle_compensate_multiple_ = 1.f;
    }
    AINFO << "RPLidar scan_mode=" << current.scan_mode
          << " max_distance=" << max_distance_
          << " Hz~" << scan_frequency;
    return true;
}

bool RpLidarSerialDriver::Start() {
    if (running_.exchange(true)) {
        return true;
    }
    if (!ConnectDevice() || !StartMotorAndScan()) {
        DisconnectDevice();
        running_ = false;
        return false;
    }
    worker_ = std::thread([this]() { CaptureLoop(); });
    return true;
}

void RpLidarSerialDriver::Stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (worker_.joinable()) {
        worker_.join();
    }
    DisconnectDevice();
}

void RpLidarSerialDriver::CaptureLoop() {
    auto* drv = static_cast<ILidarDriver*>(driver_);
    if (!drv) {
        return;
    }
    const bool inverted = hardware::ParseBool(params_, "inverted", false);
    const bool angle_compensate =
        hardware::ParseBool(params_, "angle_compensate", true);
    const float range_min =
        static_cast<float>(hardware::ParseDouble(params_, "range_min", 0.15));

    constexpr std::size_t kMaxNodes = 8192;
    std::vector<sl_lidar_response_measurement_node_hq_t> nodes(kMaxNodes);

    auto last = std::chrono::steady_clock::now();
    while (running_.load()) {
        size_t count = nodes.size();
        const sl_result op =
            drv->grabScanDataHq(nodes.data(), count, 1000);
        if (SL_IS_FAIL(op)) {
            continue;
        }
        drv->ascendScanData(nodes.data(), count);
        const auto now = std::chrono::steady_clock::now();
        const double scan_time =
            std::chrono::duration<double>(now - last).count();
        last = now;

        ConvertOptions opt;
        opt.frame_id = frame_id_;
        opt.inverted = inverted;
        opt.range_min = range_min;
        opt.range_max = max_distance_;
        opt.scan_time_s = scan_time > 0.0 ? scan_time : 0.1;

        automsgs::msgs::sensor_msgs::LaserScan scan_msg;
        if (angle_compensate) {
            const int n_comp =
                static_cast<int>(360 * angle_compensate_multiple_);
            std::vector<sl_lidar_response_measurement_node_hq_t> compensated(
                static_cast<std::size_t>(n_comp));
            std::memset(compensated.data(), 0,
                        compensated.size() * sizeof(compensated[0]));
            int offset = 0;
            for (size_t i = 0; i < count; ++i) {
                const float angle_deg =
                    nodes[i].angle_z_q14 * 90.f / 16384.f;
                int angle_value =
                    static_cast<int>(angle_deg * angle_compensate_multiple_);
                if ((angle_value - offset) < 0) {
                    offset = angle_value;
                }
                for (int j = 0; j < static_cast<int>(angle_compensate_multiple_);
                     ++j) {
                    int idx = angle_value - offset + j;
                    if (idx >= n_comp) {
                        idx = n_comp - 1;
                    }
                    if (idx >= 0) {
                        compensated[static_cast<std::size_t>(idx)] = nodes[i];
                    }
                }
            }
            scan_msg = NodesToLaserScan(compensated.data(),
                                        compensated.size(), 0.f,
                                        static_cast<float>(2.0 * M_PI), opt);
        } else {
            // Valid span between first/last non-zero distance (rplidar_ros).
            size_t start = 0;
            size_t end = count > 0 ? count - 1 : 0;
            while (start < count && nodes[start].dist_mm_q2 == 0) {
                ++start;
            }
            while (end > start && nodes[end].dist_mm_q2 == 0) {
                --end;
            }
            if (end <= start) {
                continue;
            }
            const float angle_min =
                nodes[start].angle_z_q14 * 90.f / 16384.f * static_cast<float>(M_PI) /
                180.f;
            const float angle_max =
                nodes[end].angle_z_q14 * 90.f / 16384.f * static_cast<float>(M_PI) /
                180.f;
            scan_msg = NodesToLaserScan(&nodes[start], end - start + 1,
                                        angle_min, angle_max, opt);
        }

        if (!callback_) {
            continue;
        }
        auto sample = std::make_unique<LidarScan>(
            id_, autolink::Time::Now(), std::move(scan_msg));
        sample->frame_id = frame_id_;
        callback_(std::move(sample));
    }
}

std::shared_ptr<SensorDriver> CreateRpLidarDriver(
    const SensorId& id, const hardware::DriverParams& params) {
    return std::make_shared<RpLidarSerialDriver>(id, params);
}

#endif  // AUTODRIVER_HAVE_RPLIDAR

REGISTER_LIDAR2D_BACKEND(rplidar, "rplidar",
                         autodriver::lidar::rplidar::CreateRpLidarDriver,
                         "slamtec");

}  // namespace rplidar
}  // namespace lidar
}  // namespace autodriver

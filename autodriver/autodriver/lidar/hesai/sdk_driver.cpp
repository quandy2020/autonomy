/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

/**
 * @file sdk_driver.cpp
 * @brief HesaiLidar_SDK_2.0 driver — ROS_2.0 param mapping + PointCloud2.
 */

#include "autodriver/lidar/hesai/sdk_driver.hpp"

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>

#include "autodriver/lidar/hesai/udp_driver.hpp"
#include "autodriver/lidar/point_cloud2_layout.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/time.hpp"

#ifdef AUTODRIVER_HAVE_HESAI_SDK
#include <hesai_lidar_sdk.hpp>
#endif

namespace autodriver {
namespace hardware {
namespace {

std::string ToLower(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

bool IsXt32UdpModel(const std::string& model) {
    const std::string m = ToLower(model);
    return m.empty() || m == "xt32" || m == "pandarxt" || m == "pandarxt-32" ||
           m == "pandarxt32" || m == "xt";
}

DriverParams WithUdpPortAlias(DriverParams params) {
    if (params.find("data_port") == params.end()) {
        const auto it = params.find("udp_port");
        if (it != params.end()) {
            params["data_port"] = it->second;
        }
    }
    return params;
}

#ifdef AUTODRIVER_HAVE_HESAI_SDK

int ParseHesaiSdkSourceType(const DriverParams& params) {
    const std::string dedicated =
        GetString(params, "hesai_source_type", "");
    const std::string raw =
        dedicated.empty() ? GetString(params, "source_type", "1") : dedicated;
    const std::string lower = ToLower(raw);
    if (lower == "online" || lower == "lidar" || lower == "data_from_lidar") {
        return 1;
    }
    if (lower == "pcap" || lower == "data_from_pcap") {
        return 2;
    }
    if (lower == "raw_packet" || lower == "rosbag" ||
        lower == "data_from_ros_packet") {
        return 3;
    }
    if (lower == "serial" || lower == "data_from_serial") {
        return 4;
    }
    if (lower == "tcp" || lower == "data_from_lidar_tcp") {
        return 5;
    }
    const int v = ParseInt(params, dedicated.empty() ? "source_type"
                                                     : "hesai_source_type",
                           1);
    if (v >= 1 && v <= 5) {
        return v;
    }
    return 1;
}

using hesai::lidar::DATA_FROM_LIDAR;
using hesai::lidar::DATA_FROM_LIDAR_TCP;
using hesai::lidar::DATA_FROM_PCAP;
using hesai::lidar::DATA_FROM_ROS_PACKET;
using hesai::lidar::DATA_FROM_SERIAL;
using hesai::lidar::DriverParam;
using hesai::lidar::HesaiLidarSdk;
using hesai::lidar::LidarDecodedFrame;
using hesai::lidar::LidarPointXYZIRT;
using hesai::lidar::PtcMode;
using hesai::lidar::SourceType;

SourceType ToSdkSourceType(int value) {
    switch (value) {
        case 2:
            return DATA_FROM_PCAP;
        case 3:
            return DATA_FROM_ROS_PACKET;
        case 4:
            return DATA_FROM_SERIAL;
        case 5:
            return DATA_FROM_LIDAR_TCP;
        case 1:
        default:
            return DATA_FROM_LIDAR;
    }
}

DriverParam BuildDriverParam(const DriverParams& params,
                             const std::string& frame_id) {
    DriverParam param;
    param.frame_id = frame_id;
    param.use_gpu = ParseBool(params, "use_gpu", false);

    param.input_param.source_type =
        ToSdkSourceType(ParseHesaiSdkSourceType(params));
    param.input_param.device_ip_address =
        GetString(params, "device_ip_address",
                  GetString(params, "lidar_ip", "192.168.1.201"));
    param.input_param.udp_port = static_cast<uint16_t>(
        ParseInt(params, "udp_port",
                 ParseInt(params, "data_port", 2368)));
    param.input_param.ptc_port =
        static_cast<uint16_t>(ParseInt(params, "ptc_port", 9347));
    param.input_param.multicast_ip_address =
        GetString(params, "multicast_ip_address", "");
    param.input_param.use_ptc_connected =
        ParseBool(params, "use_ptc_connected", true);
    param.input_param.host_ptc_port =
        static_cast<uint16_t>(ParseInt(params, "host_ptc_port", 0));
    param.input_param.correction_file_path = GetString(
        params, "correction_file_path",
        GetString(params, "calibration_path", ""));
    param.input_param.firetimes_path =
        GetString(params, "firetimes_path", "");
    param.input_param.host_ip_address =
        GetString(params, "host_ip_address",
                  GetString(params, "host_ip", ""));
    param.input_param.fault_message_port =
        static_cast<uint16_t>(ParseInt(params, "fault_message_port", 0));
    param.input_param.standby_mode = ParseInt(params, "standby_mode", -1);
    param.input_param.speed = ParseInt(params, "speed", -1);
    param.input_param.recv_point_cloud_timeout =
        static_cast<float>(ParseDouble(params, "recv_point_cloud_timeout", -1));
    param.input_param.ptc_connect_timeout =
        static_cast<float>(ParseDouble(params, "ptc_connect_timeout", -1));
    param.input_param.ptc_mode =
        ParseInt(params, "ptc_mode", 0) == 1 ? PtcMode::tcp_ssl : PtcMode::tcp;
    param.input_param.certFile = GetString(params, "certFile", "");
    param.input_param.privateKeyFile =
        GetString(params, "privateKeyFile", "");
    param.input_param.caFile = GetString(params, "caFile", "");
    param.input_param.pcap_path = GetString(params, "pcap_path", "");
    param.input_param.rs485_com =
        GetString(params, "rs485_com", "/dev/ttyUSB0");
    param.input_param.rs232_com =
        GetString(params, "rs232_com", "/dev/ttyUSB1");
    param.input_param.point_cloud_baudrate =
        ParseInt(params, "point_cloud_baudrate", 3125000);
    param.input_param.correction_save_path =
        GetString(params, "correction_save_path", "");
    param.input_param.device_udp_src_port =
        static_cast<uint16_t>(ParseInt(params, "device_udp_src_port", 0));
    param.input_param.device_fault_port =
        static_cast<uint16_t>(ParseInt(params, "device_fault_port", 0));
    param.input_param.frame_id = frame_id;
    param.input_param.send_point_cloud_ros = true;

    param.decoder_param.frame_start_azimuth = static_cast<float>(
        ParseDouble(params, "frame_start_azimuth", 0.0));
    param.decoder_param.use_timestamp_type = static_cast<uint16_t>(
        ParseInt(params, "use_timestamp_type", 0));
    param.decoder_param.thread_num = ParseInt(params, "thread_num", 4);
    param.decoder_param.transform_param.use_flag =
        ParseBool(params, "transform_flag", false);
    param.decoder_param.transform_param.x =
        static_cast<float>(ParseDouble(params, "x", 0.0));
    param.decoder_param.transform_param.y =
        static_cast<float>(ParseDouble(params, "y", 0.0));
    param.decoder_param.transform_param.z =
        static_cast<float>(ParseDouble(params, "z", 0.0));
    param.decoder_param.transform_param.roll =
        static_cast<float>(ParseDouble(params, "roll", 0.0));
    param.decoder_param.transform_param.pitch =
        static_cast<float>(ParseDouble(params, "pitch", 0.0));
    param.decoder_param.transform_param.yaw =
        static_cast<float>(ParseDouble(params, "yaw", 0.0));
    param.decoder_param.fov_start = ParseInt(params, "fov_start", -1);
    param.decoder_param.fov_end = ParseInt(params, "fov_end", -1);
    param.decoder_param.enable_packet_loss_tool =
        ParseBool(params, "enable_packet_loss_tool", false);
    param.decoder_param.distance_correction_flag =
        ParseBool(params, "distance_correction_flag", false);
    param.decoder_param.xt_spot_correction =
        ParseBool(params, "xt_spot_correction", false);
    param.decoder_param.channel_fov_filter_path =
        GetString(params, "channel_fov_filter_path", "");
    param.decoder_param.multi_fov_filter_ranges =
        GetString(params, "multi_fov_filter_ranges", "");
    param.decoder_param.frame_frequency =
        static_cast<float>(ParseDouble(params, "frame_frequency", 0.0));
    param.decoder_param.default_frame_frequency = static_cast<float>(
        ParseDouble(params, "default_frame_frequency", 10.0));
    param.decoder_param.echo_mode_filter = static_cast<uint16_t>(
        ParseInt(params, "echo_mode_filter", 0));
    param.decoder_param.pcap_play_synchronization =
        ParseBool(params, "pcap_play_synchronization", true);
    param.decoder_param.pcap_play_in_loop =
        ParseBool(params, "pcap_play_in_loop", false);
    param.decoder_param.play_rate_ =
        static_cast<float>(ParseDouble(params, "play_rate_", 1.0));
    // JT128 ~1.6M pts/s; sample_config default is 262144000 when non-zero.
    {
        const int sock_buf = ParseInt(params, "socket_buffer_size", 0);
        if (sock_buf > 0) {
            param.decoder_param.socket_buffer_size =
                static_cast<uint32_t>(sock_buf);
        }
    }
    param.input_param.device_tcp_src_port = static_cast<uint16_t>(
        ParseInt(params, "device_tcp_src_port", 5120));

    return param;
}

double PointTimestampToNs(double stamp) {
    // SDK / ROS2 use seconds; autodriver XYZIT stores nanoseconds as float64.
    if (stamp > 1.0e12) {
        return stamp;
    }
    return stamp * 1.0e9;
}

automsgs::msgs::sensor_msgs::PointCloud2 ConvertSdkFrameToPointCloud(
    const LidarDecodedFrame<LidarPointXYZIRT>& frame,
    const std::string& frame_id) {
    const bool multi = frame.fParam.IsMultiFrameFrequency() != 0;
    const uint32_t points_number =
        multi ? frame.multi_points_num : frame.points_num;
    const LidarPointXYZIRT* points =
        multi ? frame.multi_points : frame.points;

    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    lidar::InitXyzitCloud(&cloud, frame_id);
    if (points == nullptr || points_number == 0) {
        lidar::FinishXyzitCloud(&cloud, 0);
        return cloud;
    }

    std::string& data = *cloud.mutable_data();
    data.reserve(static_cast<std::size_t>(points_number) *
                 lidar::kXyzitPointStep);
    std::uint32_t width = 0;
    for (uint32_t i = 0; i < points_number; ++i) {
        const LidarPointXYZIRT& p = points[i];
        if (p.x == 0.f && p.y == 0.f && p.z == 0.f) {
            continue;
        }
        lidar::AppendXyzitPoint(&data, p.x, p.y, p.z,
                                static_cast<float>(p.intensity),
                                PointTimestampToNs(p.timestamp));
        ++width;
    }
    lidar::FinishXyzitCloud(&cloud, width);

    const double frame_start =
        multi ? frame.multi_frame_start_timestamp : frame.frame_start_timestamp;
    if (frame_start > 0.0) {
        const auto sec = static_cast<std::int64_t>(std::floor(frame_start));
        const auto nsec = static_cast<std::uint32_t>(
            std::llround((frame_start - static_cast<double>(sec)) * 1.0e9));
        cloud.mutable_header()->mutable_stamp()->set_sec(
            static_cast<std::int32_t>(sec));
        cloud.mutable_header()->mutable_stamp()->set_nanosec(nsec);
    }
    return cloud;
}

#endif  // AUTODRIVER_HAVE_HESAI_SDK

}  // namespace

struct HesaiSdkDriver::SdkState {
#ifdef AUTODRIVER_HAVE_HESAI_SDK
    std::unique_ptr<HesaiLidarSdk<LidarPointXYZIRT>> sdk;
#endif
};

HesaiSdkDriver::~HesaiSdkDriver() {
    Stop();
    sdk_.reset();
}

bool WantHesaiSdk(const DriverParams& params) {
    if (ParseBool(params, "use_sdk", false)) {
        return true;
    }
    const std::string parser = ToLower(GetString(params, "parser", ""));
    if (parser == "sdk" || parser == "hesai_sdk" || parser == "ros2") {
        return true;
    }
    const std::string model = GetString(params, "model", "XT32");
    return !IsXt32UdpModel(model);
}

HesaiSdkDriver::HesaiSdkDriver(SensorId id, DriverParams params)
    : id_(std::move(id)),
      params_(std::move(params)),
      sdk_(std::make_unique<SdkState>()) {
    frame_id_ = GetString(params_, "frame_id",
                          GetString(params_, "ros_frame_id", id_));
    model_ = GetString(params_, "model", "hesai");
    cloud_channel_ = GetString(params_, "channel", "");

    lidar::LidarBaseOptions options;
    options.source = lidar::SourceType::kOnline;
    options.cloud_channel = cloud_channel_;
    options.publish_scan = false;
    InitBase(options);
}

bool HesaiSdkDriver::InitSdk() {
#ifdef AUTODRIVER_HAVE_HESAI_SDK
    if (sdk_->sdk) {
        return true;
    }
    auto sdk = std::make_unique<HesaiLidarSdk<LidarPointXYZIRT>>();
    const DriverParam param = BuildDriverParam(params_, frame_id_);
    sdk->RegRecvCallback(
        [this](const LidarDecodedFrame<LidarPointXYZIRT>& frame) {
            OnPointCloudFrame(&frame);
        });
    if (!sdk->Init(param)) {
        AERROR << "Hesai SDK Init failed id=" << id_
               << " device_ip=" << param.input_param.device_ip_address
               << " udp_port=" << param.input_param.udp_port;
        return false;
    }
    sdk->Start();
    sdk_->sdk = std::move(sdk);
    sdk_owned_ = true;
    AINFO << "Hesai SDK started id=" << id_ << " model=" << model_
          << " frame_id=" << frame_id_
          << " source_type=" << ParseHesaiSdkSourceType(params_);
    return true;
#else
    AERROR << "Hesai SDK not available; install via "
              "scripts/install_hesai_lidar_sdk.sh and rebuild with "
              "-DAUTODRIVER_WITH_HESAI_SDK=ON";
    return false;
#endif
}

void HesaiSdkDriver::UninitSdk() {
#ifdef AUTODRIVER_HAVE_HESAI_SDK
    if (sdk_ && sdk_->sdk) {
        sdk_->sdk->Stop();
        sdk_->sdk.reset();
    }
#endif
    sdk_owned_ = false;
}

bool HesaiSdkDriver::Start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return true;
    }
    if (!InitSdk()) {
        running_ = false;
        return false;
    }
    return true;
}

void HesaiSdkDriver::Stop() {
    if (!running_.exchange(false)) {
        return;
    }
    UninitSdk();
}

void HesaiSdkDriver::OnPointCloudFrame(const void* frame) {
#ifdef AUTODRIVER_HAVE_HESAI_SDK
    if (!running_.load() || frame == nullptr) {
        return;
    }
    const auto* decoded =
        static_cast<const LidarDecodedFrame<LidarPointXYZIRT>*>(frame);
    auto cloud_msg = ConvertSdkFrameToPointCloud(*decoded, frame_id_);
    if (cloud_msg.width() == 0) {
        return;
    }
    auto sample = std::make_shared<LidarCloud>(
        id_, autolink::Time::Now(), std::move(cloud_msg));
    sample->frame_id = frame_id_;
    sample->channel = cloud_channel_;
    WritePointCloud(sample);
#else
    (void)frame;
#endif
}

void HesaiSdkDriver::WritePointCloud(std::shared_ptr<SensorSample> cloud) {
    SampleCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mu_);
        cb = callback_;
    }
    if (cb && cloud) {
        cb(cloud->Clone());
    }
}

SensorDriver* CreateHesaiSdkDriver(const SensorId& id,
                                   const DriverParams& params) {
    return new HesaiSdkDriver(id, params);
}

SensorDriver* CreateHesaiLidarDriver(const SensorId& id,
                                     const DriverParams& params) {
    if (WantHesaiSdk(params)) {
#ifdef AUTODRIVER_HAVE_HESAI_SDK
        return CreateHesaiSdkDriver(id, params);
#else
        AERROR << "Hesai backend requested SDK path (use_sdk/parser/model) "
                  "but AUTODRIVER_HAVE_HESAI_SDK is not set; install "
                  "HesaiLidar_SDK_2.0 via scripts/install_hesai_lidar_sdk.sh";
        return nullptr;
#endif
    }
    return CreateHesaiUdpDriver(id, WithUdpPortAlias(params));
}

}  // namespace hardware
}  // namespace autodriver

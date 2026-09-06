/*
 * Copyright 2026 Autodriver contributors
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

#include "autodriver/lidar/hesai/udp_driver.hpp"

#include <cmath>
#include <chrono>
#include <cstring>
#include <utility>

#include "autodriver/lidar/hesai/convert.hpp"
#include "autodriver/lidar/scan_cut.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {
namespace hardware {

HesaiUdpDriver::HesaiUdpDriver(SensorId id, DriverParams params)
    : id_(std::move(id)), params_(std::move(params)) {
    data_port_ = ParseInt(params_, "data_port", 2368);
    packets_per_scan_ = ParseInt(params_, "packets_per_scan", 180);
    if (packets_per_scan_ <= 0) {
        packets_per_scan_ = 180;
    }
    reconnect_attempts_ = ParseInt(params_, "reconnect_attempts", 3);
    packet_queue_capacity_ = ParseInt(params_, "packet_queue_capacity", 256);
    if (packet_queue_capacity_ <= 0) {
        packet_queue_capacity_ = 256;
    }
    packet_queue_ =
        std::make_unique<lidar::PacketQueue<lidar::hesai::PacketBuffer>>(
            static_cast<std::size_t>(packet_queue_capacity_));
    use_azimuth_cut_ = ParseBool(params_, "use_azimuth_cut", true);
    const double cut_deg = ParseDouble(params_, "scan_cut_angle_deg", 0.0);
    scan_cut_angle_centideg_ =
        static_cast<int>(std::lround(cut_deg * 100.0));
    model_ = GetString(params_, "model", "XT32");
    frame_id_ = GetString(params_, "frame_id", id_);
    bind_host_ = GetString(params_, "bind_host", "");
    enable_compensator_ = ParseBool(params_, "enable_compensator", false);

    calibration_ = lidar::hesai::DefaultXt32Calibration();
    const std::string cal_path = GetString(params_, "calibration_path", "");
    if (!cal_path.empty()) {
        lidar::hesai::BeamCalibration loaded;
        std::string err;
        if (lidar::hesai::LoadBeamCalibrationYaml(cal_path, &loaded, &err)) {
            calibration_ = std::move(loaded);
            AINFO << "Hesai loaded beam calibration " << cal_path;
        } else {
            AWARN << "Hesai calibration_path failed (" << err
                  << "); using default XT32 elevations";
        }
    }

    if (enable_compensator_) {
        lidar::CompensatorOptions opt;
        opt.world_frame_id = GetString(params_, "world_frame_id", "world");
        compensator_ = std::make_unique<lidar::MotionCompensator>(opt);
        pose_buffer_ = std::make_shared<lidar::PoseBuffer>();
        compensator_->SetPoseLookup(pose_buffer_->AsLookup());
    }

    lidar::LidarBaseOptions options;
    options.source = lidar::ParseSourceType(
        GetString(params_, "source_type", "online"));
    options.cloud_channel = GetString(params_, "channel", "");
    options.scan_channel = GetString(params_, "scan_channel", "");
    options.publish_scan = ParseBool(params_, "publish_scan", false);
    InitBase(options);
}

HesaiUdpDriver::~HesaiUdpDriver() { Stop(); }

bool HesaiUdpDriver::InitPacket() {
    if (options().source == lidar::SourceType::kRawPacket) {
        return true;
    }
    stream_ = common::CreateUdpStream(bind_host_, data_port_);
    return stream_ != nullptr;
}

void HesaiUdpDriver::WriteScan(std::shared_ptr<SensorSample> scan) {
    if (callback_ && scan) {
        callback_(std::unique_ptr<SensorSample>(scan.release()));
    }
}

void HesaiUdpDriver::WritePointCloud(std::shared_ptr<SensorSample> cloud) {
    if (callback_ && cloud) {
        callback_(std::unique_ptr<SensorSample>(cloud.release()));
    }
}

bool HesaiUdpDriver::Start() {
    if (running_.exchange(true)) {
        return true;
    }
    if (!InitPacket()) {
        running_ = false;
        return false;
    }
    last_azimuth_centideg_ = -1;
    if (options().source == lidar::SourceType::kOnline) {
        if (!stream_->Connect()) {
            AERROR << "Hesai UDP bind failed on port " << data_port_ << ": "
                   << stream_->last_error();
            if (!common::ReconnectStream(stream_.get(), reconnect_attempts_)) {
                running_ = false;
                stream_.reset();
                return false;
            }
        }
        processor_ = std::thread([this]() { ProcessLoop(); });
        reader_ = std::thread([this]() { ReadLoop(); });
    }
    AINFO << "HesaiUdpDriver started id=" << id_ << " model=" << model_
          << " source="
          << (options().source == lidar::SourceType::kOnline ? "online"
                                                             : "raw_packet")
          << " azimuth_cut=" << use_azimuth_cut_
          << " publish_scan=" << options().publish_scan;
    return true;
}

void HesaiUdpDriver::Stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (stream_) {
        stream_->Disconnect();
    }
    if (reader_.joinable()) {
        reader_.join();
    }
    if (processor_.joinable()) {
        processor_.join();
    }
    stream_.reset();
    if (packet_queue_) {
        packet_queue_->Clear();
    }
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan_.clear();
    last_azimuth_centideg_ = -1;
}

bool HesaiUdpDriver::IsRunning() const { return running_.load(); }

void HesaiUdpDriver::SetSampleCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

void HesaiUdpDriver::SetPoseLookup(lidar::PoseLookup lookup) {
    if (compensator_) {
        compensator_->SetPoseLookup(std::move(lookup));
    }
}

void HesaiUdpDriver::PushPose(std::uint64_t time_ns,
                              const Eigen::Affine3d& pose) {
    if (pose_buffer_) {
        pose_buffer_->Push(time_ns, pose);
    }
}

void HesaiUdpDriver::PushRawPacket(const std::uint8_t* data, std::size_t size) {
    if (!lidar::hesai::IsXt32PointCloudPacket(data, size)) {
        return;
    }
    lidar::hesai::PacketBuffer packet{};
    std::memcpy(packet.data(), data, lidar::hesai::kPacketSize);
    HandlePacket(packet);
}

void HesaiUdpDriver::PushScan(std::shared_ptr<SensorSample> scan) {
    InjectScan(std::move(scan));
}

void HesaiUdpDriver::ReadScanCallback(std::shared_ptr<SensorSample> scan) {
    const auto* packet_scan = dynamic_cast<const LidarPacketScan*>(scan.get());
    if (packet_scan == nullptr) {
        return;
    }
    const std::size_t step = packet_scan->packet_bytes();
    if (step != lidar::hesai::kPacketSize ||
        packet_scan->payload().size() < step) {
        AWARN << "Hesai ReadScanCallback bad packet size id=" << id_;
        return;
    }
    lidar::hesai::ScanPackets packets;
    const auto& payload = packet_scan->payload();
    packets.reserve(payload.size() / step);
    for (std::size_t off = 0; off + step <= payload.size(); off += step) {
        if (!lidar::hesai::IsXt32PointCloudPacket(payload.data() + off,
                                                  step)) {
            continue;
        }
        lidar::hesai::PacketBuffer packet{};
        std::memcpy(packet.data(), payload.data() + off, step);
        packets.push_back(packet);
    }
    if (packets.empty()) {
        return;
    }
    ConvertAndPublish(packets);
}

void HesaiUdpDriver::ReadLoop() {
    lidar::hesai::PacketBuffer packet{};
    while (running_.load()) {
        if (!stream_) {
            break;
        }
        if (stream_->status() == common::Stream::Status::kError) {
            if (!common::ReconnectStream(stream_.get(), reconnect_attempts_)) {
                AERROR << "Hesai UDP reconnect failed: "
                       << stream_->last_error();
                break;
            }
        }
        const std::size_t n =
            stream_->Read(packet.data(), packet.size(), 200);
        if (n == 0) {
            continue;
        }
        if (n != lidar::hesai::kPacketSize ||
            !lidar::hesai::IsXt32PointCloudPacket(packet.data(), n)) {
            continue;
        }
        packet_queue_->Push(packet);
    }
}

void HesaiUdpDriver::ProcessLoop() {
    while (running_.load()) {
        auto item = packet_queue_->TryPop();
        if (!item) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        HandlePacket(*item);
    }
}

void HesaiUdpDriver::HandlePacket(const lidar::hesai::PacketBuffer& packet) {
    int curr_az = 0;
    const bool have_az =
        lidar::HesaiLastAzimuthCentideg(packet.data(), packet.size(), &curr_az);
    bool complete = false;
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        scan_.push_back(packet);
        const int count = static_cast<int>(scan_.size());
        if (have_az) {
            complete = lidar::ShouldEmitScan(
                use_azimuth_cut_, last_azimuth_centideg_, curr_az,
                scan_cut_angle_centideg_, count, packets_per_scan_);
            last_azimuth_centideg_ = curr_az;
        } else {
            complete = count >= packets_per_scan_;
        }
    }
    if (complete) {
        EmitScan();
    }
}

void HesaiUdpDriver::EmitScan() {
    lidar::hesai::ScanPackets packets;
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        packets.swap(scan_);
    }
    if (packets.empty()) {
        return;
    }

    if (options().publish_scan && callback_) {
        std::vector<std::uint8_t> payload;
        payload.reserve(packets.size() * lidar::hesai::kPacketSize);
        for (const auto& pkt : packets) {
            payload.insert(payload.end(), pkt.begin(), pkt.end());
        }
        auto scan = std::make_shared<LidarPacketScan>(
            id_, autolink::Time::Now(), std::move(payload),
            lidar::hesai::kPacketSize);
        scan->frame_id = frame_id_;
        scan->channel = options().scan_channel;
        WriteScan(scan);
    }

    ConvertAndPublish(packets);
}

void HesaiUdpDriver::ConvertAndPublish(
    const lidar::hesai::ScanPackets& packets) {
    if (!callback_ || packets.empty()) {
        return;
    }

    auto cloud_msg = lidar::hesai::ConvertPacketsToPointCloud(
        packets, frame_id_, calibration_);
    if (compensator_) {
        automsgs::msgs::sensor_msgs::PointCloud2 compensated;
        if (compensator_->Compensate(cloud_msg, &compensated)) {
            cloud_msg = std::move(compensated);
        } else {
            AWARN << "Hesai motion compensation skipped id=" << id_;
        }
    }
    auto sample = std::make_shared<LidarCloud>(
        id_, autolink::Time::Now(), std::move(cloud_msg));
    sample->frame_id = frame_id_;
    sample->channel = options().cloud_channel;
    WritePointCloud(sample);
}

std::shared_ptr<SensorDriver> CreateHesaiUdpDriver(const SensorId& id,
                                                   const DriverParams& params) {
    return std::make_shared<HesaiUdpDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

#include "autodriver/lidar/backend_register.hpp"

REGISTER_LIDAR_BACKEND(hesai, "hesai",
                       autodriver::hardware::CreateHesaiUdpDriver, "pandar");

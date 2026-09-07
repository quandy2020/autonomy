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

#include "autodriver/lidar/velodyne/udp_driver.hpp"

#include <cmath>
#include <chrono>
#include <cstring>
#include <utility>

#include "autodriver/lidar/scan_cut.hpp"
#include "autodriver/lidar/velodyne/convert.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {
namespace hardware {

VelodyneUdpDriver::VelodyneUdpDriver(SensorId id, DriverParams params)
    : id_(std::move(id)), params_(std::move(params)) {
    data_port_ = ParseInt(params_, "data_port", 2368);
    packets_per_scan_ = ParseInt(params_, "packets_per_scan", 75);
    if (packets_per_scan_ <= 0) {
        packets_per_scan_ = 75;
    }
    reconnect_attempts_ = ParseInt(params_, "reconnect_attempts", 3);
    packet_queue_capacity_ = ParseInt(params_, "packet_queue_capacity", 256);
    if (packet_queue_capacity_ <= 0) {
        packet_queue_capacity_ = 256;
    }
    packet_queue_ =
        std::make_unique<lidar::PacketQueue<lidar::velodyne::PacketBuffer>>(
            static_cast<std::size_t>(packet_queue_capacity_));
    use_azimuth_cut_ = ParseBool(params_, "use_azimuth_cut", true);
    const double cut_deg = ParseDouble(params_, "scan_cut_angle_deg", 0.0);
    scan_cut_angle_centideg_ =
        static_cast<int>(std::lround(cut_deg * 100.0));
    model_ = GetString(params_, "model", "VLP-16");
    frame_id_ = GetString(params_, "frame_id", id_);
    bind_host_ = GetString(params_, "bind_host", "");
    enable_compensator_ = ParseBool(params_, "enable_compensator", false);

    calibration_ = lidar::velodyne::DefaultVlp16Calibration();
    const std::string cal_path = GetString(params_, "calibration_path", "");
    if (!cal_path.empty()) {
        lidar::velodyne::BeamCalibration loaded;
        std::string err;
        if (lidar::velodyne::LoadBeamCalibrationYaml(cal_path, &loaded,
                                                     &err)) {
            calibration_ = std::move(loaded);
            AINFO << "Velodyne loaded beam calibration " << cal_path
                  << " lasers=" << calibration_.vert_correction_rad.size();
        } else {
            AWARN << "Velodyne calibration_path failed (" << err
                  << "); using default VLP-16";
        }
    } else if (model_ != "VLP-16" && model_ != "VLP16") {
        AWARN << "Velodyne model \"" << model_
              << "\" without calibration_path; using VLP-16 angles";
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

VelodyneUdpDriver::~VelodyneUdpDriver() { Stop(); }

bool VelodyneUdpDriver::InitPacket() {
    if (options().source == lidar::SourceType::kRawPacket) {
        return true;
    }
    stream_ = common::CreateUdpStream(bind_host_, data_port_);
    return stream_ != nullptr;
}

void VelodyneUdpDriver::WriteScan(std::shared_ptr<SensorSample> scan) {
    if (callback_ && scan) {
        callback_(std::unique_ptr<SensorSample>(scan.release()));
    }
}

void VelodyneUdpDriver::WritePointCloud(std::shared_ptr<SensorSample> cloud) {
    if (callback_ && cloud) {
        callback_(std::unique_ptr<SensorSample>(cloud.release()));
    }
}

bool VelodyneUdpDriver::Start() {
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
            AERROR << "Velodyne UDP bind failed on port " << data_port_ << ": "
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
    AINFO << "VelodyneUdpDriver started id=" << id_
          << " source=" << (options().source == lidar::SourceType::kOnline
                                ? "online"
                                : "raw_packet")
          << " azimuth_cut=" << use_azimuth_cut_
          << " publish_scan=" << options().publish_scan;
    return true;
}

void VelodyneUdpDriver::Stop() {
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

bool VelodyneUdpDriver::IsRunning() const { return running_.load(); }

void VelodyneUdpDriver::SetSampleCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

void VelodyneUdpDriver::SetPoseLookup(lidar::PoseLookup lookup) {
    if (compensator_) {
        compensator_->SetPoseLookup(std::move(lookup));
    }
}

void VelodyneUdpDriver::PushPose(std::uint64_t time_ns,
                                 const Eigen::Affine3d& pose) {
    if (pose_buffer_) {
        pose_buffer_->Push(time_ns, pose);
    }
}

void VelodyneUdpDriver::PushRawPacket(const std::uint8_t* data,
                                      std::size_t size) {
    if (data == nullptr || size < lidar::velodyne::kFiringPacketSize) {
        return;
    }
    lidar::velodyne::PacketBuffer packet{};
    std::memcpy(packet.data(), data, lidar::velodyne::kFiringPacketSize);
    HandlePacket(packet);
}

void VelodyneUdpDriver::PushScan(std::shared_ptr<SensorSample> scan) {
    InjectScan(std::move(scan));
}

void VelodyneUdpDriver::ReadScanCallback(std::shared_ptr<SensorSample> scan) {
    const auto* packet_scan = dynamic_cast<const LidarPacketScan*>(scan.get());
    if (packet_scan == nullptr) {
        return;
    }
    const std::size_t step = packet_scan->packet_bytes();
    if (step != lidar::velodyne::kFiringPacketSize ||
        packet_scan->payload().size() < step) {
        AWARN << "Velodyne ReadScanCallback bad packet size id=" << id_;
        return;
    }
    lidar::velodyne::ScanPackets packets;
    const auto& payload = packet_scan->payload();
    packets.reserve(payload.size() / step);
    for (std::size_t off = 0; off + step <= payload.size(); off += step) {
        lidar::velodyne::PacketBuffer packet{};
        std::memcpy(packet.data(), payload.data() + off, step);
        packets.push_back(packet);
    }
    if (packets.empty()) {
        return;
    }
    ConvertAndPublish(packets);
}

void VelodyneUdpDriver::ReadLoop() {
    lidar::velodyne::PacketBuffer packet{};
    while (running_.load()) {
        if (!stream_) {
            break;
        }
        if (stream_->status() == common::Stream::Status::kError) {
            if (!common::ReconnectStream(stream_.get(), reconnect_attempts_)) {
                AERROR << "Velodyne UDP reconnect failed: "
                       << stream_->last_error();
                break;
            }
        }
        const std::size_t n =
            stream_->Read(packet.data(), packet.size(), 200);
        if (n == 0) {
            continue;
        }
        if (n != lidar::velodyne::kFiringPacketSize) {
            continue;
        }
        packet_queue_->Push(packet);
    }
}

void VelodyneUdpDriver::ProcessLoop() {
    while (running_.load()) {
        auto item = packet_queue_->TryPop();
        if (!item) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        HandlePacket(*item);
    }
}

void VelodyneUdpDriver::HandlePacket(
    const lidar::velodyne::PacketBuffer& packet) {
    int curr_az = 0;
    const bool have_az = lidar::VelodyneLastAzimuthCentideg(
        packet.data(), packet.size(), &curr_az);
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

void VelodyneUdpDriver::EmitScan() {
    lidar::velodyne::ScanPackets packets;
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        packets.swap(scan_);
    }
    if (packets.empty()) {
        return;
    }

    if (options().publish_scan && callback_) {
        std::vector<std::uint8_t> payload;
        payload.reserve(packets.size() * lidar::velodyne::kFiringPacketSize);
        for (const auto& pkt : packets) {
            payload.insert(payload.end(), pkt.begin(), pkt.end());
        }
        auto scan = std::make_shared<LidarPacketScan>(
            id_, autolink::Time::Now(), std::move(payload),
            lidar::velodyne::kFiringPacketSize);
        scan->frame_id = frame_id_;
        scan->channel = options().scan_channel;
        WriteScan(scan);
    }

    ConvertAndPublish(packets);
}

void VelodyneUdpDriver::ConvertAndPublish(
    const lidar::velodyne::ScanPackets& packets) {
    if (!callback_ || packets.empty()) {
        return;
    }

    auto cloud_msg = lidar::velodyne::ConvertPacketsToPointCloud(
        packets, frame_id_, calibration_);
    if (compensator_) {
        automsgs::msgs::sensor_msgs::PointCloud2 compensated;
        if (compensator_->Compensate(cloud_msg, &compensated)) {
            cloud_msg = std::move(compensated);
        } else {
            AWARN << "Velodyne motion compensation skipped id=" << id_;
        }
    }
    auto sample = std::make_shared<LidarCloud>(
        id_, autolink::Time::Now(), std::move(cloud_msg));
    sample->frame_id = frame_id_;
    sample->channel = options().cloud_channel;
    WritePointCloud(sample);
}

std::shared_ptr<SensorDriver> CreateVelodyneUdpDriver(
    const SensorId& id, const DriverParams& params) {
    return std::make_shared<VelodyneUdpDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

#include "autodriver/lidar/backend_register.hpp"

REGISTER_LIDAR_BACKEND(velodyne, "velodyne",
                       autodriver::hardware::CreateVelodyneUdpDriver, "udp");

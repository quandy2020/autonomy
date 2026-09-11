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
 * @file udp_scan_driver_base.hpp
 * @brief CRTP base for UDP lidar drivers: Read → Queue → Cut → Convert → Publish.
 *
 * Traits must provide PacketBuffer, ScanPackets, BeamCalibration, packet size,
 * AcceptPacket, LastAzimuthCentideg, DefaultCalibration, LoadCalibration,
 * ConvertPackets, and default YAML knobs. Livox / RPLidar do not use this path.
 */

#ifndef AUTODRIVER_LIDAR_UDP_SCAN_DRIVER_BASE_HPP_
#define AUTODRIVER_LIDAR_UDP_SCAN_DRIVER_BASE_HPP_

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autodriver/common/stream.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/lidar/lidar_component_base.hpp"
#include "autodriver/lidar/motion_compensator.hpp"
#include "autodriver/lidar/motion_pose_sink.hpp"
#include "autodriver/lidar/packet_queue.hpp"
#include "autodriver/lidar/pose_buffer.hpp"
#include "autodriver/lidar/scan_cut.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/time.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace lidar {

/**
 * @class autodriver::lidar::UdpScanDriverBase
 * @brief Shared online / RAW_PACKET pipeline for Velodyne- and Hesai-style UDP lidars.
 *
 * @tparam Derived Concrete driver (CRTP).
 * @tparam Traits Vendor packet / convert / calibration hooks.
 *
 * Traits requirements (static):
 * - `using PacketBuffer`, `ScanPackets`, `BeamCalibration`
 * - `kPacketSize`, `kDefaultPacketsPerScan`, `kDefaultDataPort`
 * - `kDefaultModel`, `kDefaultFrameId`, `kLogTag` (C string)
 * - `AcceptPacket(const uint8_t*, size_t) -> bool`
 * - `LastAzimuthCentideg(const PacketBuffer&, int*) -> bool`
 * - `DefaultCalibration() -> BeamCalibration`
 * - `LoadCalibration(path, out, err) -> bool`
 * - `WarnUnknownModel(const string& model)` (optional no-op ok)
 * - `ConvertPackets(packets, frame_id, cal) -> PointCloud2`
 */
template <typename Derived, typename Traits>
class UdpScanDriverBase : public SensorDriver,
                          public LidarComponentBase,
                          public MotionPoseSink {
public:
    /**
     * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(UdpScanDriverBase)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(UdpScanDriverBase)

    using PacketBuffer = typename Traits::PacketBuffer;
    using ScanPackets = typename Traits::ScanPackets;
    using BeamCalibration = typename Traits::BeamCalibration;

    /**
     * @brief Parse common YAML params, load calibration via Traits, InitBase.
     * @param[in] id Sensor instance id.
     * @param[in] params DriverParams (cold-path parse only).
     */
    UdpScanDriverBase(SensorId id, hardware::DriverParams params)
        : id_(std::move(id)), params_(std::move(params)) {
        using hardware::GetString;
        using hardware::ParseBool;
        using hardware::ParseDouble;
        using hardware::ParseInt;

        data_port_ = ParseInt(params_, "data_port", Traits::kDefaultDataPort);
        packets_per_scan_ =
            ParseInt(params_, "packets_per_scan", Traits::kDefaultPacketsPerScan);
        if (packets_per_scan_ <= 0) {
            packets_per_scan_ = Traits::kDefaultPacketsPerScan;
        }
        reconnect_attempts_ = ParseInt(params_, "reconnect_attempts", 3);
        packet_queue_capacity_ =
            ParseInt(params_, "packet_queue_capacity", 256);
        if (packet_queue_capacity_ <= 0) {
            packet_queue_capacity_ = 256;
        }
        packet_queue_ = std::make_unique<PacketQueue<PacketBuffer>>(
            static_cast<std::size_t>(packet_queue_capacity_));
        use_azimuth_cut_ = ParseBool(params_, "use_azimuth_cut", true);
        const double cut_deg = ParseDouble(params_, "scan_cut_angle_deg", 0.0);
        scan_cut_angle_centideg_ =
            static_cast<int>(std::lround(cut_deg * 100.0));
        model_ = GetString(params_, "model", Traits::kDefaultModel);
        frame_id_ = GetString(params_, "frame_id", id_);
        bind_host_ = GetString(params_, "bind_host", "");
        enable_compensator_ = ParseBool(params_, "enable_compensator", false);

        calibration_ = Traits::DefaultCalibration();
        const std::string cal_path = GetString(params_, "calibration_path", "");
        if (!cal_path.empty()) {
            BeamCalibration loaded;
            std::string err;
            if (Traits::LoadCalibration(cal_path, &loaded, &err)) {
                calibration_ = std::move(loaded);
                AINFO << Traits::kLogTag << " loaded beam calibration "
                      << cal_path;
            } else {
                AWARN << Traits::kLogTag << " calibration_path failed (" << err
                      << "); using default";
            }
        } else {
            Traits::WarnUnknownModel(model_);
        }

        if (enable_compensator_) {
            CompensatorOptions opt;
            opt.world_frame_id = GetString(params_, "world_frame_id", "world");
            compensator_ = std::make_unique<MotionCompensator>(opt);
            pose_buffer_ = std::make_shared<PoseBuffer>();
            compensator_->SetPoseLookup(pose_buffer_->AsLookup());
        }

        LidarBaseOptions options;
        options.source =
            ParseSourceType(GetString(params_, "source_type", "online"));
        options.cloud_channel = GetString(params_, "channel", "");
        options.scan_channel = GetString(params_, "scan_channel", "");
        options.publish_scan = ParseBool(params_, "publish_scan", false);
        InitBase(options);
    }

    /**
     * @brief Stop capture threads on destruction.
     */
    ~UdpScanDriverBase() override { Stop(); }

    /**
     * @brief Report sensor modality.
     * @return SensorType::kLidar3d.
     */
    SensorType GetSensorType() const override { return SensorType::kLidar3d; }

    /**
     * @brief Configured sensor instance id.
     * @return Reference to the id passed at construction.
     */
    const SensorId& GetSensorId() const override { return id_; }

    /**
     * @brief Bind UDP (online) and start read/process threads.
     * @return true when started or already running; false on bind failure.
     */
    bool Start() override {
        if (running_.exchange(true)) {
            return true;
        }
        if (!InitPacket()) {
            running_ = false;
            return false;
        }
        last_azimuth_centideg_ = -1;
        if (options().source == SourceType::kOnline) {
            if (!stream_->Connect()) {
                AERROR << Traits::kLogTag << " UDP bind failed on port "
                       << data_port_ << ": " << stream_->last_error();
                if (!common::ReconnectStream(stream_.get(),
                                             reconnect_attempts_)) {
                    running_ = false;
                    stream_.reset();
                    return false;
                }
            }
            processor_ = std::thread([this]() { ProcessLoop(); });
            reader_ = std::thread([this]() { ReadLoop(); });
        }
        AINFO << Traits::kLogTag << " started id=" << id_ << " model=" << model_
              << " source="
              << (options().source == SourceType::kOnline ? "online"
                                                          : "raw_packet")
              << " azimuth_cut=" << use_azimuth_cut_
              << " publish_scan=" << options().publish_scan;
        return true;
    }

    /**
     * @brief Stop threads, clear queues, and close the stream.
     */
    void Stop() override {
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

    /**
     * @brief Whether capture threads are active.
     * @return true while Start succeeded and Stop has not completed.
     */
    bool IsRunning() const override { return running_.load(); }

    /**
     * @brief Register the sample sink for scan / cloud samples.
     * @param[in] callback Invoked with owning sample clones.
     */
    void SetSampleCallback(SampleCallback callback) override {
        callback_ = std::move(callback);
    }

    /**
     * @brief Install pose lookup used by the motion compensator.
     * @param[in] lookup PoseLookup callable; no-op when compensator is disabled.
     */
    void SetPoseLookup(PoseLookup lookup) override {
        if (compensator_) {
            compensator_->SetPoseLookup(std::move(lookup));
        }
    }

    /**
     * @brief Push a timed world←lidar pose into the internal PoseBuffer.
     * @param[in] time_ns Pose timestamp in nanoseconds.
     * @param[in] pose Affine transform at @p time_ns.
     */
    void PushPose(std::uint64_t time_ns, const Eigen::Affine3d& pose) override {
        if (pose_buffer_) {
            pose_buffer_->Push(time_ns, pose);
        }
    }

    /**
     * @brief Shared pose buffer used when enable_compensator is true.
     * @return Shared PoseBuffer, or nullptr when compensator is disabled.
     */
    PoseBuffer::SharedPtr pose_buffer() const override { return pose_buffer_; }

    /**
     * @brief CRTP access to the concrete driver.
     * @return Reference to @p Derived.
     */
    Derived& self() { return static_cast<Derived&>(*this); }

    /**
     * @brief Const CRTP access to the concrete driver.
     * @return Const reference to @p Derived.
     */
    const Derived& self() const {
        return static_cast<const Derived&>(*this);
    }

    /**
     * @brief Inject one vendor packet (online aggregation / unit tests).
     * @param[in] data Packet bytes.
     * @param[in] size Byte length; must pass Traits::AcceptPacket.
     */
    void PushRawPacket(const std::uint8_t* data, std::size_t size) {
        if (!Traits::AcceptPacket(data, size)) {
            return;
        }
        PacketBuffer packet{};
        std::memcpy(packet.data(), data, Traits::kPacketSize);
        HandlePacket(packet);
    }

    /**
     * @brief Replay one recorded LidarPacketScan (Convert → cloud only).
     * @param[in] scan Aggregated scan sample.
     */
    void PushScan(std::shared_ptr<SensorSample> scan) {
        InjectScan(std::move(scan));
    }

protected:
    bool InitPacket() override {
        if (options().source == SourceType::kRawPacket) {
            return true;
        }
        stream_ = common::CreateUdpStream(bind_host_, data_port_);
        return stream_ != nullptr;
    }

    void WriteScan(std::shared_ptr<SensorSample> scan) override {
        if (callback_ && scan) {
            callback_(scan->Clone());
        }
    }

    void WritePointCloud(std::shared_ptr<SensorSample> cloud) override {
        if (callback_ && cloud) {
            callback_(cloud->Clone());
        }
    }

    void ReadScanCallback(std::shared_ptr<SensorSample> scan) override {
        const auto* packet_scan =
            dynamic_cast<const LidarPacketScan*>(scan.get());
        if (packet_scan == nullptr) {
            return;
        }
        const std::size_t step = packet_scan->packet_bytes();
        if (step != Traits::kPacketSize ||
            packet_scan->payload().size() < step) {
            AWARN << Traits::kLogTag
                  << " ReadScanCallback bad packet size id=" << id_;
            return;
        }
        ScanPackets packets;
        const auto& payload = packet_scan->payload();
        packets.reserve(payload.size() / step);
        for (std::size_t off = 0; off + step <= payload.size(); off += step) {
            if (!Traits::AcceptPacket(payload.data() + off, step)) {
                continue;
            }
            PacketBuffer packet{};
            std::memcpy(packet.data(), payload.data() + off, step);
            packets.push_back(packet);
        }
        if (packets.empty()) {
            return;
        }
        ConvertAndPublish(packets);
    }

    /**
     * @brief Access Traits calibration held by the base.
     * @return Const reference to the loaded / default BeamCalibration.
     */
    const BeamCalibration& calibration() const { return calibration_; }

    /**
     * @brief Sensor instance id.
     * @return Const reference to @p id_.
     */
    const SensorId& id() const { return id_; }

    /**
     * @brief PointCloud2 / scan frame_id.
     * @return Const reference to @p frame_id_.
     */
    const std::string& frame_id() const { return frame_id_; }

    /**
     * @brief YAML model string.
     * @return Const reference to @p model_.
     */
    const std::string& model() const { return model_; }

private:
    void ReadLoop() {
        PacketBuffer packet{};
        while (running_.load()) {
            if (!stream_) {
                break;
            }
            if (stream_->status() == common::Stream::Status::kError) {
                if (!common::ReconnectStream(stream_.get(),
                                             reconnect_attempts_)) {
                    AERROR << Traits::kLogTag << " UDP reconnect failed: "
                           << stream_->last_error();
                    break;
                }
            }
            const std::size_t n =
                stream_->Read(packet.data(), packet.size(), 200);
            if (n == 0) {
                continue;
            }
            if (!Traits::AcceptPacket(packet.data(), n)) {
                continue;
            }
            packet_queue_->Push(packet);
        }
    }

    void ProcessLoop() {
        while (running_.load()) {
            auto item = packet_queue_->WaitPop(std::chrono::milliseconds(1));
            if (!item) {
                continue;
            }
            HandlePacket(*item);
        }
    }

    void HandlePacket(const PacketBuffer& packet) {
        int curr_az = 0;
        const bool have_az = Traits::LastAzimuthCentideg(packet, &curr_az);
        bool complete = false;
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            scan_.push_back(packet);
            const int count = static_cast<int>(scan_.size());
            if (have_az) {
                complete = ShouldEmitScan(
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

    void EmitScan() {
        ScanPackets packets;
        {
            std::lock_guard<std::mutex> lock(scan_mutex_);
            packets.swap(scan_);
        }
        if (packets.empty()) {
            return;
        }

        if (options().publish_scan && callback_) {
            std::vector<std::uint8_t> payload;
            payload.reserve(packets.size() * Traits::kPacketSize);
            for (const auto& pkt : packets) {
                payload.insert(payload.end(), pkt.begin(), pkt.end());
            }
            auto scan = std::make_shared<LidarPacketScan>(
                id_, autolink::Time::Now(), std::move(payload),
                Traits::kPacketSize);
            scan->frame_id = frame_id_;
            scan->channel = options().scan_channel;
            WriteScan(scan);
        }

        ConvertAndPublish(packets);
    }

    void ConvertAndPublish(const ScanPackets& packets) {
        if (!callback_ || packets.empty()) {
            return;
        }

        auto cloud_msg =
            Traits::ConvertPackets(packets, frame_id_, calibration_);
        if (compensator_) {
            automsgs::msgs::sensor_msgs::PointCloud2 compensated;
            if (compensator_->Compensate(cloud_msg, &compensated)) {
                cloud_msg = std::move(compensated);
            } else {
                AWARN << Traits::kLogTag
                      << " motion compensation skipped id=" << id_;
            }
        }
        auto sample = std::make_shared<LidarCloud>(
            id_, autolink::Time::Now(), std::move(cloud_msg));
        sample->frame_id = frame_id_;
        sample->channel = options().cloud_channel;
        WritePointCloud(sample);
    }

    SensorId id_;
    hardware::DriverParams params_;
    SampleCallback callback_;
    std::unique_ptr<common::Stream> stream_{nullptr};
    std::atomic<bool> running_{false};
    std::thread reader_;
    std::thread processor_;
    std::unique_ptr<PacketQueue<PacketBuffer>> packet_queue_{nullptr};

    int data_port_ = Traits::kDefaultDataPort;
    int packets_per_scan_ = Traits::kDefaultPacketsPerScan;
    int reconnect_attempts_ = 3;
    int packet_queue_capacity_ = 256;
    bool use_azimuth_cut_ = true;
    int scan_cut_angle_centideg_ = 0;
    int last_azimuth_centideg_ = -1;
    std::string model_{Traits::kDefaultModel};
    std::string frame_id_{Traits::kDefaultFrameId};
    std::string bind_host_;
    bool enable_compensator_ = false;
    std::unique_ptr<MotionCompensator> compensator_{nullptr};
    PoseBuffer::SharedPtr pose_buffer_{nullptr};
    BeamCalibration calibration_{};

    std::mutex scan_mutex_;
    ScanPackets scan_;
};

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_UDP_SCAN_DRIVER_BASE_HPP_

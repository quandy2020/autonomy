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

/**
 * @file
 * @brief Hesai PandarXT-32 UDP driver (Scan → Convert → Compensate).
 */

#ifndef AUTODRIVER_LIDAR_HESAI_UDP_DRIVER_HPP_
#define AUTODRIVER_LIDAR_HESAI_UDP_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Geometry>

#include "autodriver/common/stream.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/lidar/hesai/calibration.hpp"
#include "autodriver/lidar/hesai/packet.hpp"
#include "autodriver/lidar/lidar_component_base.hpp"
#include "autodriver/lidar/motion_compensator.hpp"
#include "autodriver/lidar/motion_pose_sink.hpp"
#include "autodriver/lidar/packet_queue.hpp"
#include "autodriver/lidar/pose_buffer.hpp"
#include "autodriver/sensor_driver.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::HesaiUdpDriver
 * @brief Binds UDP data port, aggregates XT32 packets, converts to PointCloud2.
 *
 * Online: ReadLoop → PacketQueue → ProcessLoop. RAW_PACKET: PushRawPacket or
 * PushScan / ReadScanCallback (aggregated scan → Convert only).
 * Default model: XT32 (PandarXT). Unknown models fall back to XT32 angles.
 */
class HesaiUdpDriver : public SensorDriver,
                       public lidar::LidarComponentBase,
                       public lidar::MotionPoseSink {
public:
    HesaiUdpDriver(SensorId id, DriverParams params);
    ~HesaiUdpDriver() override;

    SensorType GetType() const override { return SensorType::kLidar3d; }
    const SensorId& GetSensorId() const override { return id_; }

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;
    void SetSampleCallback(SampleCallback callback) override;

    void SetPoseLookup(lidar::PoseLookup lookup) override;
    void PushPose(std::uint64_t time_ns, const Eigen::Affine3d& pose) override;
    std::shared_ptr<lidar::PoseBuffer> pose_buffer() const override {
        return pose_buffer_;
    }

    void PushRawPacket(const std::uint8_t* data, std::size_t size);
    void PushScan(std::shared_ptr<SensorSample> scan);

protected:
    bool InitPacket() override;
    void WriteScan(std::shared_ptr<SensorSample> scan) override;
    void WritePointCloud(std::shared_ptr<SensorSample> cloud) override;
    void ReadScanCallback(std::shared_ptr<SensorSample> scan) override;

private:
    void ReadLoop();
    void ProcessLoop();
    void HandlePacket(const lidar::hesai::PacketBuffer& packet);
    void EmitScan();
    void ConvertAndPublish(const lidar::hesai::ScanPackets& packets);

    SensorId id_;
    DriverParams params_;
    SampleCallback callback_;
    std::unique_ptr<common::Stream> stream_;
    std::atomic<bool> running_{false};
    std::thread reader_;
    std::thread processor_;
    std::unique_ptr<lidar::PacketQueue<lidar::hesai::PacketBuffer>>
        packet_queue_;

    int data_port_ = 2368;
    int packets_per_scan_ = 180;
    int reconnect_attempts_ = 3;
    int packet_queue_capacity_ = 256;
    bool use_azimuth_cut_ = true;
    int scan_cut_angle_centideg_ = 0;
    int last_azimuth_centideg_ = -1;
    std::string model_ = "XT32";
    std::string frame_id_ = "hesai";
    std::string bind_host_;
    bool enable_compensator_ = false;
    std::unique_ptr<lidar::MotionCompensator> compensator_;
    std::shared_ptr<lidar::PoseBuffer> pose_buffer_;
    lidar::hesai::BeamCalibration calibration_;

    std::mutex scan_mutex_;
    lidar::hesai::ScanPackets scan_;
};

std::shared_ptr<SensorDriver> CreateHesaiUdpDriver(const SensorId& id,
                                                   const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_HESAI_UDP_DRIVER_HPP_

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
 * @brief Velodyne UDP online / raw-packet driver (Scan → Convert → Compensate).
 */

#ifndef AUTODRIVER_LIDAR_VELODYNE_UDP_DRIVER_HPP_
#define AUTODRIVER_LIDAR_VELODYNE_UDP_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "autodriver/common/stream.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/lidar/lidar_component_base.hpp"
#include "autodriver/lidar/motion_compensator.hpp"
#include "autodriver/lidar/motion_pose_sink.hpp"
#include "autodriver/lidar/packet_queue.hpp"
#include "autodriver/lidar/pose_buffer.hpp"
#include "autodriver/lidar/velodyne/calibration.hpp"
#include "autodriver/lidar/velodyne/packet.hpp"
#include "autodriver/sensor_driver.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::VelodyneUdpDriver
 * @brief Binds UDP data port, aggregates firing packets, converts to PointCloud2.
 *
 * Online path: ReadLoop → PacketQueue → ProcessLoop → HandlePacket.
 * RAW_PACKET: PushRawPacket (per packet) or PushScan / ReadScanCallback
 * (aggregated LidarPacketScan → Convert only).
 * Scan cut: azimuth wrap (default) with packets_per_scan as max safety.
 */
class VelodyneUdpDriver : public SensorDriver,
                          public lidar::LidarComponentBase,
                          public lidar::MotionPoseSink {
public:
    VelodyneUdpDriver(SensorId id, DriverParams params);
    ~VelodyneUdpDriver() override;

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

    /** Inject one firing packet (online aggregation / unit tests). */
    void PushRawPacket(const std::uint8_t* data, std::size_t size);

    /**
     * @brief Replay one recorded LidarPacketScan (Convert → cloud only).
     * Equivalent to ReadScanCallback for external callers.
     */
    void PushScan(std::shared_ptr<SensorSample> scan);

protected:
    bool InitPacket() override;
    void WriteScan(std::shared_ptr<SensorSample> scan) override;
    void WritePointCloud(std::shared_ptr<SensorSample> cloud) override;
    void ReadScanCallback(std::shared_ptr<SensorSample> scan) override;

private:
    void ReadLoop();
    void ProcessLoop();
    void HandlePacket(const lidar::velodyne::PacketBuffer& packet);
    void EmitScan();
    void ConvertAndPublish(const lidar::velodyne::ScanPackets& packets);

    SensorId id_;
    DriverParams params_;
    SampleCallback callback_;
    std::unique_ptr<common::Stream> stream_;
    std::atomic<bool> running_{false};
    std::thread reader_;
    std::thread processor_;
    std::unique_ptr<lidar::PacketQueue<lidar::velodyne::PacketBuffer>>
        packet_queue_;

    int data_port_ = 2368;
    int packets_per_scan_ = 75;
    int reconnect_attempts_ = 3;
    int packet_queue_capacity_ = 256;
    bool use_azimuth_cut_ = true;
    int scan_cut_angle_centideg_ = 0;
    int last_azimuth_centideg_ = -1;
    std::string model_ = "VLP-16";
    std::string frame_id_ = "velodyne";
    std::string bind_host_;
    bool enable_compensator_ = false;
    std::unique_ptr<lidar::MotionCompensator> compensator_;
    std::shared_ptr<lidar::PoseBuffer> pose_buffer_;
    lidar::velodyne::BeamCalibration calibration_;

    std::mutex scan_mutex_;
    lidar::velodyne::ScanPackets scan_;
};

std::shared_ptr<SensorDriver> CreateVelodyneUdpDriver(
    const SensorId& id, const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_VELODYNE_UDP_DRIVER_HPP_

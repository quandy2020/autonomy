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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#include "autodriver/canbus/can_receiver.hpp"
#include "autodriver/canbus/protocol_data.hpp"
#include "autodriver/common/stream.hpp"
#include "autodriver/common/status.hpp"
#include "autodriver/lidar/hesai/convert.hpp"
#include "autodriver/lidar/hesai/packet.hpp"
#include "autodriver/lidar/hesai/udp_driver.hpp"
#include "autodriver/lidar/backend_registry.hpp"
#include "autodriver/lidar/lidar_component_base.hpp"
#include "autodriver/lidar/scan_cut.hpp"
#include "autodriver/lidar/source_type.hpp"
#include "autodriver/lidar/velodyne/convert.hpp"
#include "autodriver/lidar/velodyne/packet.hpp"
#include "autodriver/lidar/velodyne/udp_driver.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/time/time.hpp"

namespace {

autodriver::lidar::hesai::PacketBuffer MakeXt32PacketWithOnePoint() {
    autodriver::lidar::hesai::PacketBuffer buf{};
    buf[0] = 0xEE;
    buf[1] = 0xFF;
    buf[2] = 0x06;  // protocol major
    buf[3] = 0x01;  // protocol minor
    buf[6] = 0x20;  // laser num
    buf[7] = 0x08;  // block num
    buf[9] = 0x04;  // dis unit 4 mm
    // Block 0 at body offset 12: azimuth 0, channel 0 distance=250 → 1.0 m
    const std::size_t ch0 = autodriver::lidar::hesai::kBodyOffset + 2;
    buf[ch0] = 250;
    buf[ch0 + 1] = 0;
    buf[ch0 + 2] = 128;  // reflectivity
    return buf;
}

}  // namespace


TEST(Stream, CreateSerialStreamDefaultsDisconnected) {
    auto stream = autodriver::common::CreateSerialStream("/dev/null", 115200);
    ASSERT_NE(stream, nullptr);
    EXPECT_EQ(stream->status(),
              autodriver::diagnostics::DeviceStatus::kDisconnected);
}

TEST(Stream, CreateUdpStreamBindLocalhostEphemeral) {
    // Bind an ephemeral port on loopback; Connect should succeed.
    auto stream = autodriver::common::CreateUdpStream("127.0.0.1", 0);
    // Port 0 may fail bind on some stacks; use a high port instead.
    stream = autodriver::common::CreateUdpStream("127.0.0.1", 23690);
    ASSERT_NE(stream, nullptr);
    EXPECT_TRUE(stream->Connect());
    EXPECT_EQ(stream->status(), autodriver::diagnostics::DeviceStatus::kOk);
    stream->Disconnect();
}

TEST(LidarBase, ParseSourceType) {
    using autodriver::lidar::ParseSourceType;
    using autodriver::lidar::SourceType;
    EXPECT_EQ(ParseSourceType("online"), SourceType::kOnline);
    EXPECT_EQ(ParseSourceType("raw_packet"), SourceType::kRawPacket);
    EXPECT_EQ(ParseSourceType("RAW_PACKET"), SourceType::kRawPacket);
}

TEST(Diagnostics, ToString) {
    using autodriver::diagnostics::DeviceStatus;
    using autodriver::diagnostics::ToString;
    EXPECT_STREQ(ToString(DeviceStatus::kOk), "ok");
    EXPECT_STREQ(ToString(DeviceStatus::kDisconnected), "disconnected");
    EXPECT_STREQ(ToString(DeviceStatus::kError), "error");
}

TEST(LidarBackendRegistry, VelodyneRegistered) {
    auto& reg = autodriver::lidar::LidarBackendRegistry::Instance();
    EXPECT_TRUE(reg.Has("velodyne"));
    EXPECT_TRUE(reg.Has("udp"));
}

TEST(VelodyneConvert, EmptyScanYieldsEmptyCloud) {
    autodriver::lidar::velodyne::ScanPackets packets;
    const auto cloud =
        autodriver::lidar::velodyne::ConvertPacketsToPointCloud(packets);
    EXPECT_EQ(cloud.width(), 0u);
    EXPECT_EQ(cloud.fields_size(), 5);
    EXPECT_EQ(cloud.point_step(), 24u);
}

TEST(VelodyneConvert, SyntheticPacketProducesPoints) {
    autodriver::lidar::velodyne::PacketBuffer buf{};
    // One block with non-zero distance on channel 0.
    buf[0] = 0xFF;
    buf[1] = 0xEE;
    buf[2] = 0;  // azimuth
    buf[3] = 0;
    buf[4] = 100;  // distance LSB (200mm)
    buf[5] = 0;
    buf[6] = 128;  // intensity
    autodriver::lidar::velodyne::ScanPackets packets{buf};
    const auto cloud =
        autodriver::lidar::velodyne::ConvertPacketsToPointCloud(packets,
                                                                "velodyne");
    EXPECT_GT(cloud.width(), 0u);
    EXPECT_EQ(cloud.point_step(), 24u);
    bool has_timestamp = false;
    for (const auto& field : cloud.fields()) {
        if (field.name() == "timestamp") {
            has_timestamp = true;
            EXPECT_EQ(field.offset(), 16u);
        }
    }
    EXPECT_TRUE(has_timestamp);
}

TEST(VelodyneDriver, RawPacketEmitsCloud) {
    autodriver::hardware::DriverParams params;
    params["source_type"] = "raw_packet";
    params["packets_per_scan"] = "1";
    params["frame_id"] = "velo";
    autodriver::hardware::VelodyneUdpDriver driver("lidar/vlp16", params);
    int clouds = 0;
    driver.SetSampleCallback([&](std::unique_ptr<autodriver::SensorSample>) {
        ++clouds;
    });
    ASSERT_TRUE(driver.Start());
    autodriver::lidar::velodyne::PacketBuffer buf{};
    buf[4] = 50;
    buf[5] = 0;
    buf[6] = 10;
    driver.PushRawPacket(buf.data(), buf.size());
    EXPECT_EQ(clouds, 1);
    driver.Stop();
}

TEST(VelodyneDriver, AzimuthWrapEmitsBeforeMaxPackets) {
    autodriver::hardware::DriverParams params;
    params["source_type"] = "raw_packet";
    params["packets_per_scan"] = "100";
    params["use_azimuth_cut"] = "true";
    params["scan_cut_angle_deg"] = "0";
    params["frame_id"] = "velo";
    params["channel"] = "/lidar/vlp16/points";
    autodriver::hardware::VelodyneUdpDriver driver("lidar/vlp16", params);
    int clouds = 0;
    std::string channel;
    driver.SetSampleCallback([&](std::unique_ptr<autodriver::SensorSample> s) {
        if (auto* cloud = dynamic_cast<autodriver::LidarCloud*>(s.get())) {
            ++clouds;
            channel = cloud->channel;
        }
    });
    ASSERT_TRUE(driver.Start());

    autodriver::lidar::velodyne::PacketBuffer high{};
    const std::size_t last = 11 * 100;
    high[last] = 0xFF;
    high[last + 1] = 0xEE;
    high[last + 2] = static_cast<std::uint8_t>(35000 & 0xFF);
    high[last + 3] = static_cast<std::uint8_t>((35000 >> 8) & 0xFF);

    autodriver::lidar::velodyne::PacketBuffer low{};
    low[last] = 0xFF;
    low[last + 1] = 0xEE;
    low[last + 2] = static_cast<std::uint8_t>(100 & 0xFF);
    low[last + 3] = 0;

    driver.PushRawPacket(high.data(), high.size());
    EXPECT_EQ(clouds, 0);
    driver.PushRawPacket(low.data(), low.size());
    EXPECT_EQ(clouds, 1);
    driver.Stop();
}

TEST(VelodyneDriver, PushScanConvertsWithoutReemittingScan) {
    autodriver::hardware::DriverParams params;
    params["source_type"] = "raw_packet";
    params["packets_per_scan"] = "100";
    params["use_azimuth_cut"] = "false";
    params["publish_scan"] = "true";
    params["frame_id"] = "velo";
    autodriver::hardware::VelodyneUdpDriver driver("lidar/vlp16", params);
    int scans = 0;
    int clouds = 0;
    driver.SetSampleCallback([&](std::unique_ptr<autodriver::SensorSample> s) {
        if (dynamic_cast<autodriver::LidarPacketScan*>(s.get()) != nullptr) {
            ++scans;
        } else if (dynamic_cast<autodriver::LidarCloud*>(s.get()) != nullptr) {
            ++clouds;
        }
    });
    ASSERT_TRUE(driver.Start());

    autodriver::lidar::velodyne::PacketBuffer buf{};
    buf[4] = 50;
    buf[5] = 0;
    buf[6] = 10;
    std::vector<std::uint8_t> payload(buf.begin(), buf.end());
    auto scan = std::make_shared<autodriver::LidarPacketScan>(
        "lidar/vlp16", autolink::Time::Now(), std::move(payload),
        autodriver::lidar::velodyne::kFiringPacketSize);
    driver.PushScan(scan);
    EXPECT_EQ(scans, 0);
    EXPECT_EQ(clouds, 1);
    driver.Stop();
}

namespace {

struct DummyImu {
    double ax = 0;
};

class AccelProtocol : public autodriver::canbus::ProtocolData<DummyImu> {
public:
    std::uint32_t can_id() const override { return 0x100; }
    bool Parse(const autodriver::io::CanFrame& frame,
               DummyImu* msg) const override {
        if (frame.dlc < 2 || msg == nullptr) {
            return false;
        }
        msg->ax = frame.data[0] + frame.data[1] * 256.0;
        return true;
    }
};

}  // namespace

TEST(Stream, CreateTcpStreamDefaultsDisconnected) {
    auto stream = autodriver::common::CreateTcpStream("127.0.0.1", 9);
    ASSERT_NE(stream, nullptr);
    EXPECT_EQ(stream->status(),
              autodriver::diagnostics::DeviceStatus::kDisconnected);
}

TEST(LidarBackendRegistry, HesaiRegistered) {
    auto& reg = autodriver::lidar::LidarBackendRegistry::Instance();
    EXPECT_TRUE(reg.Has("hesai"));
    EXPECT_TRUE(reg.Has("pandar"));
    autodriver::hardware::DriverParams params;
    params["source_type"] = "raw_packet";
    auto driver = reg.Create("hesai", "lidar/hesai", params);
    ASSERT_NE(driver, nullptr);
    EXPECT_EQ(driver->GetType(), autodriver::SensorType::kLidar3d);
}

TEST(LidarBackendRegistry, VendorStubsRegistered) {
    auto& reg = autodriver::lidar::LidarBackendRegistry::Instance();
    EXPECT_TRUE(reg.Has("livox"));
    EXPECT_TRUE(reg.Has("rslidar"));
    EXPECT_TRUE(reg.Has("robosense"));
    EXPECT_TRUE(reg.Has("lslidar"));
    EXPECT_TRUE(reg.Has("seyond"));
    EXPECT_TRUE(reg.Has("vanjee"));
    EXPECT_TRUE(reg.Has("vanjeelidar"));
    EXPECT_EQ(reg.Create("livox", "lidar/x", {}), nullptr);
}

TEST(ScanCut, CrossedCutAngleDetectsWrapAtZero) {
    using autodriver::lidar::CrossedCutAngle;
    EXPECT_FALSE(CrossedCutAngle(-1, 100, 0));
    EXPECT_FALSE(CrossedCutAngle(100, 200, 0));
    EXPECT_TRUE(CrossedCutAngle(35900, 100, 0));
    EXPECT_TRUE(CrossedCutAngle(17000, 19000, 18000));
    EXPECT_FALSE(CrossedCutAngle(19000, 20000, 18000));
}

TEST(ScanCut, ShouldEmitScanRespectsMaxPackets) {
    using autodriver::lidar::ShouldEmitScan;
    EXPECT_TRUE(ShouldEmitScan(true, 100, 200, 0, 75, 75));
    EXPECT_FALSE(ShouldEmitScan(true, 100, 200, 0, 10, 75));
    EXPECT_TRUE(ShouldEmitScan(false, 35900, 100, 0, 75, 75));
    EXPECT_FALSE(ShouldEmitScan(false, 35900, 100, 0, 10, 75));
}

TEST(HesaiConvert, EmptyScanYieldsEmptyCloud) {
    autodriver::lidar::hesai::ScanPackets packets;
    const auto cloud =
        autodriver::lidar::hesai::ConvertPacketsToPointCloud(packets);
    EXPECT_EQ(cloud.width(), 0u);
    EXPECT_EQ(cloud.fields_size(), 5);
    EXPECT_EQ(cloud.point_step(), 24u);
}

TEST(HesaiConvert, SyntheticPacketProducesPoints) {
    autodriver::lidar::hesai::ScanPackets packets{MakeXt32PacketWithOnePoint()};
    const auto cloud = autodriver::lidar::hesai::ConvertPacketsToPointCloud(
        packets, "hesai", "XT32");
    EXPECT_GT(cloud.width(), 0u);
    EXPECT_EQ(cloud.point_step(), 24u);
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    std::memcpy(&x, cloud.data().data() + 0, 4);
    std::memcpy(&y, cloud.data().data() + 4, 4);
    std::memcpy(&z, cloud.data().data() + 8, 4);
    // Channel 0 elev=+15°, azimuth=0 → x≈0, y≈cos(15), z≈sin(15)
    EXPECT_NEAR(x, 0.0f, 1e-3f);
    EXPECT_NEAR(y, static_cast<float>(std::cos(15.0 * 0.017453292519943295)),
                1e-3f);
    EXPECT_NEAR(z, static_cast<float>(std::sin(15.0 * 0.017453292519943295)),
                1e-3f);
}

TEST(HesaiDriver, PublishScanEmitsPacketScanAndCloud) {
    autodriver::hardware::DriverParams params;
    params["source_type"] = "raw_packet";
    params["packets_per_scan"] = "1";
    params["frame_id"] = "hesai";
    params["publish_scan"] = "true";
    params["model"] = "XT32";
    autodriver::hardware::HesaiUdpDriver driver("lidar/hesai", params);
    int scans = 0;
    int clouds = 0;
    driver.SetSampleCallback([&](std::unique_ptr<autodriver::SensorSample> s) {
        if (dynamic_cast<autodriver::LidarPacketScan*>(s.get()) != nullptr) {
            ++scans;
        } else if (dynamic_cast<autodriver::LidarCloud*>(s.get()) != nullptr) {
            ++clouds;
        }
    });
    ASSERT_TRUE(driver.Start());
    const auto buf = MakeXt32PacketWithOnePoint();
    driver.PushRawPacket(buf.data(), buf.size());
    EXPECT_EQ(scans, 1);
    EXPECT_EQ(clouds, 1);
    driver.Stop();
}

TEST(VelodyneDriver, PublishScanEmitsPacketScanAndCloud) {
    autodriver::hardware::DriverParams params;
    params["source_type"] = "raw_packet";
    params["packets_per_scan"] = "1";
    params["frame_id"] = "velo";
    params["publish_scan"] = "true";
    autodriver::hardware::VelodyneUdpDriver driver("lidar/vlp16", params);
    int scans = 0;
    int clouds = 0;
    driver.SetSampleCallback([&](std::unique_ptr<autodriver::SensorSample> s) {
        if (dynamic_cast<autodriver::LidarPacketScan*>(s.get()) != nullptr) {
            ++scans;
        } else if (dynamic_cast<autodriver::LidarCloud*>(s.get()) != nullptr) {
            ++clouds;
        }
    });
    ASSERT_TRUE(driver.Start());
    autodriver::lidar::velodyne::PacketBuffer buf{};
    buf[4] = 50;
    buf[5] = 0;
    buf[6] = 10;
    driver.PushRawPacket(buf.data(), buf.size());
    EXPECT_EQ(scans, 1);
    EXPECT_EQ(clouds, 1);
    driver.Stop();
}

TEST(CanProtocol, MessageManagerDispatchesById) {
    autodriver::canbus::MessageManager<DummyImu> manager;
    manager.Register(std::make_shared<AccelProtocol>());
    DummyImu last{};
    int count = 0;
    manager.SetPublishCallback([&](const DummyImu& msg) {
        last = msg;
        ++count;
    });
    autodriver::io::CanFrame frame{};
    frame.id = 0x100;
    frame.dlc = 2;
    frame.data[0] = 10;
    frame.data[1] = 1;
    EXPECT_TRUE(manager.Parse(frame));
    EXPECT_EQ(count, 1);
    EXPECT_EQ(last.ax, 10 + 256.0);
    frame.id = 0x101;
    EXPECT_FALSE(manager.Parse(frame));
}

TEST(CanReceiver, ConstructAndManagerRegister) {
    autodriver::canbus::CanReceiver<DummyImu> receiver;
    receiver.manager().Register(std::make_shared<AccelProtocol>());
    EXPECT_EQ(receiver.manager().size(), 1u);
    EXPECT_FALSE(receiver.IsRunning());
}

TEST(CanProtocol, DualIdFusionLikeImu) {
    // Mirrors CanImuDriver: two ProtocolData → one publish callback that
    // requires both kinds before emitting.
    struct Event {
        enum class Kind { kA, kB };
        Kind kind = Kind::kA;
        int value = 0;
    };
    class ProtoA : public autodriver::canbus::ProtocolData<Event> {
    public:
        std::uint32_t can_id() const override { return 0x100; }
        bool Parse(const autodriver::io::CanFrame& frame,
                   Event* msg) const override {
            msg->kind = Event::Kind::kA;
            msg->value = frame.data[0];
            return true;
        }
    };
    class ProtoB : public autodriver::canbus::ProtocolData<Event> {
    public:
        std::uint32_t can_id() const override { return 0x101; }
        bool Parse(const autodriver::io::CanFrame& frame,
                   Event* msg) const override {
            msg->kind = Event::Kind::kB;
            msg->value = frame.data[0];
            return true;
        }
    };

    autodriver::canbus::MessageManager<Event> manager;
    manager.Register(std::make_shared<ProtoA>());
    manager.Register(std::make_shared<ProtoB>());
    int fused = 0;
    int a = -1;
    int b = -1;
    bool have_a = false;
    bool have_b = false;
    manager.SetPublishCallback([&](const Event& e) {
        if (e.kind == Event::Kind::kA) {
            a = e.value;
            have_a = true;
        } else {
            b = e.value;
            have_b = true;
        }
        if (have_a && have_b) {
            ++fused;
            have_a = false;
            have_b = false;
        }
    });

    autodriver::io::CanFrame frame{};
    frame.dlc = 1;
    frame.id = 0x100;
    frame.data[0] = 3;
    EXPECT_TRUE(manager.Parse(frame));
    EXPECT_EQ(fused, 0);
    frame.id = 0x101;
    frame.data[0] = 7;
    EXPECT_TRUE(manager.Parse(frame));
    EXPECT_EQ(fused, 1);
    EXPECT_EQ(a, 3);
    EXPECT_EQ(b, 7);
}

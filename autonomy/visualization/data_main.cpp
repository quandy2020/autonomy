/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <signal.h>

#include <chrono>
#include <cmath>
#include <thread>

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autolink/message/message_traits.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/version.hpp"
#include "autonomy/commsgs/proto/builtin_interfaces.pb.h"
#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "autonomy/commsgs/proto/map_msgs.pb.h"
#include "autonomy/commsgs/proto/planning_msgs.pb.h"
#include "autonomy/commsgs/proto/sensor_msgs.pb.h"
#include "autonomy/commsgs/proto/std_msgs.pb.h"

using autolink::Rate;
using autolink::Time;

namespace autonomy {
namespace visualization {
namespace {

std::shared_ptr<autolink::Node> g_node;
bool g_running = true;

void SignalHandler(int) {
    AINFO << "Shutting down data publisher...";
    g_running = false;
    exit(0);
}

// 获取当前时间戳
autonomy::commsgs::proto::builtin_interfaces::Time GetCurrentTime() {
    auto now = Time::Now();
    int64_t nanos = now.ToNanosecond();
    autonomy::commsgs::proto::builtin_interfaces::Time time;
    time.set_sec(static_cast<int32_t>(nanos / 1000000000LL));
    time.set_nanosec(static_cast<uint32_t>(nanos % 1000000000ULL));
    return time;
}

// 创建 Header
autonomy::commsgs::proto::std_msgs::Header CreateHeader(
    const std::string& frame_id) {
    autonomy::commsgs::proto::std_msgs::Header header;
    *header.mutable_stamp() = GetCurrentTime();
    header.set_frame_id(frame_id);
    return header;
}

// 发布 IMU 数据
void PublishImu(
    std::shared_ptr<
        autolink::Writer<autonomy::commsgs::proto::sensor_msgs::Imu>>
        writer,
    uint64_t seq) {
    auto msg = std::make_shared<autonomy::commsgs::proto::sensor_msgs::Imu>();

    // 设置 header
    *msg->mutable_header() = CreateHeader("map");

    // 设置方向（四元数）
    auto* orientation = msg->mutable_orientation();
    orientation->set_x(0.0);
    orientation->set_y(0.0);
    orientation->set_z(0.0);
    orientation->set_w(1.0);

    // 设置角速度（模拟旋转）
    auto* angular_vel = msg->mutable_angular_velocity();
    angular_vel->set_x(0.1 * std::sin(seq * 0.1));
    angular_vel->set_y(0.1 * std::cos(seq * 0.1));
    angular_vel->set_z(0.05);

    // 设置线性加速度（模拟运动）
    auto* linear_acc = msg->mutable_linear_acceleration();
    linear_acc->set_x(0.5 * std::sin(seq * 0.2));
    linear_acc->set_y(0.3 * std::cos(seq * 0.15));
    linear_acc->set_z(9.81);  // 重力加速度

    if (writer->Write(msg)) {
        AINFO << "Published IMU message, seq=" << seq;
    }
}

// 发布 Camera (Image) 数据
void PublishImage(
    std::shared_ptr<
        autolink::Writer<autonomy::commsgs::proto::sensor_msgs::Image>>
        writer,
    uint64_t seq) {
    auto msg = std::make_shared<autonomy::commsgs::proto::sensor_msgs::Image>();

    // 设置 header
    *msg->mutable_header() = CreateHeader("map");

    // 设置图像参数
    constexpr int WIDTH = 640;
    constexpr int HEIGHT = 480;
    msg->set_width(WIDTH);
    msg->set_height(HEIGHT);
    msg->set_encoding("rgb8");
    msg->set_step(WIDTH * 3);  // RGB = 3 bytes per pixel
    msg->set_is_bigendian(false);

    // 生成模拟图像数据（彩色渐变）
    int data_size = WIDTH * HEIGHT * 3;
    std::string image_data;
    image_data.reserve(data_size);
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            uint8_t r = static_cast<uint8_t>((x * 255) / WIDTH);
            uint8_t g = static_cast<uint8_t>((y * 255) / HEIGHT);
            uint8_t b = static_cast<uint8_t>((seq * 50) % 255);
            image_data.push_back(static_cast<char>(r));
            image_data.push_back(static_cast<char>(g));
            image_data.push_back(static_cast<char>(b));
        }
    }
    msg->set_data(image_data);

    if (writer->Write(msg)) {
        AINFO << "Published Image message, seq=" << seq
              << ", size=" << data_size << " bytes";
    }
}

// 发布 Range 数据
void PublishRange(
    std::shared_ptr<
        autolink::Writer<autonomy::commsgs::proto::sensor_msgs::Range>>
        writer,
    uint64_t seq) {
    auto msg = std::make_shared<autonomy::commsgs::proto::sensor_msgs::Range>();

    // 设置 header
    *msg->mutable_header() = CreateHeader("map");

    // 设置范围参数
    // 0 = ULTRASOUND, 1 = INFRARED
    msg->set_radiation_type(0);   // ULTRASOUND
    msg->set_field_of_view(0.5);  // 30度视场角
    msg->set_min_range(0.1);
    msg->set_max_range(10.0);

    // 模拟距离变化
    double range = 2.0 + 1.5 * std::sin(seq * 0.1);
    msg->set_range(range);

    if (writer->Write(msg)) {
        AINFO << "Published Range message, seq=" << seq << ", range=" << range
              << "m";
    }
}

// 发布点云 (PointCloud2) 数据
void PublishPointCloud2(
    std::shared_ptr<
        autolink::Writer<autonomy::commsgs::proto::sensor_msgs::PointCloud2>>
        writer,
    uint64_t seq) {
    auto msg =
        std::make_shared<autonomy::commsgs::proto::sensor_msgs::PointCloud2>();

    // 设置 header
    *msg->mutable_header() = CreateHeader("map");

    // 设置点云参数
    constexpr int WIDTH = 100;
    constexpr int HEIGHT = 1;
    msg->set_width(WIDTH);
    msg->set_height(HEIGHT);
    msg->set_is_bigendian(false);
    msg->set_is_dense(false);

    // 定义字段：x, y, z, intensity
    constexpr int POINT_SIZE = 16;  // 4 floats (x,y,z) + 1 uint32 (intensity)

    // 添加字段定义
    auto* field_x = msg->add_fields();
    field_x->set_name("x");
    field_x->set_offset(0);
    field_x->set_datatype(7);  // FLOAT32
    field_x->set_count(1);

    auto* field_y = msg->add_fields();
    field_y->set_name("y");
    field_y->set_offset(4);
    field_y->set_datatype(7);  // FLOAT32
    field_y->set_count(1);

    auto* field_z = msg->add_fields();
    field_z->set_name("z");
    field_z->set_offset(8);
    field_z->set_datatype(7);  // FLOAT32
    field_z->set_count(1);

    auto* field_i = msg->add_fields();
    field_i->set_name("intensity");
    field_i->set_offset(12);
    field_i->set_datatype(6);  // UINT32
    field_i->set_count(1);

    msg->set_point_step(POINT_SIZE);
    msg->set_row_step(WIDTH * POINT_SIZE);

    // 生成点云数据（圆形点云）
    for (int i = 0; i < WIDTH; ++i) {
        double angle = 2.0 * M_PI * i / WIDTH + seq * 0.05;
        double radius = 5.0;

        float x = static_cast<float>(radius * std::cos(angle));
        float y = static_cast<float>(radius * std::sin(angle));
        float z = static_cast<float>(1.0 + 0.5 * std::sin(seq * 0.1));
        uint32_t intensity = 100 + static_cast<uint32_t>(50 * std::sin(angle));

        // PointCloud2 的 data 是 repeated uint32，每个 uint32 代表一个字节
        // 按字节添加数据
        const uint8_t* x_bytes = reinterpret_cast<const uint8_t*>(&x);
        const uint8_t* y_bytes = reinterpret_cast<const uint8_t*>(&y);
        const uint8_t* z_bytes = reinterpret_cast<const uint8_t*>(&z);
        const uint8_t* i_bytes = reinterpret_cast<const uint8_t*>(&intensity);

        for (int j = 0; j < 4; ++j)
            msg->mutable_data()->Add(static_cast<uint32_t>(x_bytes[j]));
        for (int j = 0; j < 4; ++j)
            msg->mutable_data()->Add(static_cast<uint32_t>(y_bytes[j]));
        for (int j = 0; j < 4; ++j)
            msg->mutable_data()->Add(static_cast<uint32_t>(z_bytes[j]));
        for (int j = 0; j < 4; ++j)
            msg->mutable_data()->Add(static_cast<uint32_t>(i_bytes[j]));
    }

    if (writer->Write(msg)) {
        AINFO << "Published PointCloud2 message, seq=" << seq
              << ", points=" << WIDTH;
    }
}

// 发布点云 (PointCloud) 数据
void PublishPointCloud(
    std::shared_ptr<
        autolink::Writer<autonomy::commsgs::proto::sensor_msgs::PointCloud>>
        writer,
    uint64_t seq) {
    auto msg =
        std::make_shared<autonomy::commsgs::proto::sensor_msgs::PointCloud>();

    // 设置 header
    *msg->mutable_header() = CreateHeader("map");

    // 生成点云数据（直线点云）
    constexpr int NUM_POINTS = 50;
    for (int i = 0; i < NUM_POINTS; ++i) {
        auto* point = msg->add_points();
        point->set_x(static_cast<float>(i * 0.1));
        point->set_y(
            static_cast<float>(2.0 + 0.5 * std::sin(i * 0.2 + seq * 0.1)));
        point->set_z(static_cast<float>(1.0));
        point->set_intensity(static_cast<uint32_t>(100 + i * 2));
        *point->mutable_timestamp() = GetCurrentTime();
    }

    if (writer->Write(msg)) {
        AINFO << "Published PointCloud message, seq=" << seq
              << ", points=" << NUM_POINTS;
    }
}

// 发布轨迹 (Path) 数据
void PublishPath(
    std::shared_ptr<
        autolink::Writer<autonomy::commsgs::proto::planning_msgs::Path>>
        writer,
    uint64_t seq) {
    auto msg =
        std::make_shared<autonomy::commsgs::proto::planning_msgs::Path>();

    // 设置 header
    *msg->mutable_header() = CreateHeader("map");

    // 生成路径点（螺旋路径）
    constexpr int NUM_POINTS = 20;
    for (int i = 0; i < NUM_POINTS; ++i) {
        auto* pose_stamped = msg->add_poses();
        *pose_stamped->mutable_header() = CreateHeader("map");

        double angle = 2.0 * M_PI * i / NUM_POINTS + seq * 0.05;
        double radius = 5.0 * (1.0 + i * 0.1);

        auto* pose = pose_stamped->mutable_pose();
        auto* position = pose->mutable_position();
        position->set_x(radius * std::cos(angle));
        position->set_y(radius * std::sin(angle));
        position->set_z(0.5);

        auto* orientation = pose->mutable_orientation();
        orientation->set_x(0.0);
        orientation->set_y(0.0);
        orientation->set_z(std::sin(angle / 2.0));
        orientation->set_w(std::cos(angle / 2.0));
    }

    if (writer->Write(msg)) {
        AINFO << "Published Path message, seq=" << seq
              << ", waypoints=" << NUM_POINTS;
    }
}

// 发布地图 (OccupancyGrid) 数据
void PublishOccupancyGrid(
    std::shared_ptr<
        autolink::Writer<autonomy::commsgs::proto::map_msgs::OccupancyGrid>>
        writer,
    uint64_t seq) {
    auto msg =
        std::make_shared<autonomy::commsgs::proto::map_msgs::OccupancyGrid>();

    // 设置 header
    *msg->mutable_header() = CreateHeader("map");

    // 设置地图参数
    constexpr int MAP_WIDTH = 100;
    constexpr int MAP_HEIGHT = 100;
    constexpr float RESOLUTION = 0.1f;  // 0.1m per cell

    auto* info = msg->mutable_info();
    info->set_width(MAP_WIDTH);
    info->set_height(MAP_HEIGHT);
    info->set_resolution(RESOLUTION);

    // 设置原点
    auto* origin = info->mutable_origin();
    auto* origin_pos = origin->mutable_position();
    origin_pos->set_x(-MAP_WIDTH * RESOLUTION / 2.0);
    origin_pos->set_y(-MAP_HEIGHT * RESOLUTION / 2.0);
    origin_pos->set_z(0.0);
    auto* origin_ori = origin->mutable_orientation();
    origin_ori->set_x(0.0);
    origin_ori->set_y(0.0);
    origin_ori->set_z(0.0);
    origin_ori->set_w(1.0);

    *info->mutable_map_load_time() = GetCurrentTime();

    // 生成地图数据（移动的障碍物）
    int data_size = MAP_WIDTH * MAP_HEIGHT;
    msg->mutable_data()->Reserve(data_size);
    for (int y = 0; y < MAP_HEIGHT; ++y) {
        for (int x = 0; x < MAP_WIDTH; ++x) {
            double cx = MAP_WIDTH / 2.0 + 20 * std::cos(seq * 0.1);
            double cy = MAP_HEIGHT / 2.0 + 20 * std::sin(seq * 0.1);
            double dist = std::sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));

            int32_t value;
            if (dist < 5) {
                value = 100;  // 占用
            } else if (dist < 8) {
                value = 50;  // 未知
            } else {
                value = 0;  // 空闲
            }
            msg->mutable_data()->Add(value);
        }
    }

    if (writer->Write(msg)) {
        AINFO << "Published OccupancyGrid message, seq=" << seq
              << ", size=" << MAP_WIDTH << "x" << MAP_HEIGHT;
    }
}

void Run() {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    autonomy::common::ShowVersion();
    AINFO << "Autonomy Data Publisher - Publishing sensor data via autolink";

    // 创建节点（autolink 已在 main 中初始化）
    g_node = autolink::CreateNode("data_publisher_node");
    if (!g_node) {
        AERROR << "Failed to create autolink node";
        return;
    }

    // 创建 writers（显式设置 message_type，避免在 topology 中 message_type
    // 为空）
    using autolink::message::MessageType;
    using autolink::proto::RoleAttributes;

    // IMU writer
    RoleAttributes imu_attr;
    imu_attr.set_channel_name("/sensor/imu");
    imu_attr.set_message_type(
        MessageType<autonomy::commsgs::proto::sensor_msgs::Imu>());
    auto imu_writer =
        g_node->CreateWriter<autonomy::commsgs::proto::sensor_msgs::Imu>(
            imu_attr);

    // Image writer
    RoleAttributes image_attr;
    image_attr.set_channel_name("/sensor/camera");
    image_attr.set_message_type(
        MessageType<autonomy::commsgs::proto::sensor_msgs::Image>());
    auto image_writer =
        g_node->CreateWriter<autonomy::commsgs::proto::sensor_msgs::Image>(
            image_attr);

    // Range writer
    RoleAttributes range_attr;
    range_attr.set_channel_name("/sensor/range");
    range_attr.set_message_type(
        MessageType<autonomy::commsgs::proto::sensor_msgs::Range>());
    auto range_writer =
        g_node->CreateWriter<autonomy::commsgs::proto::sensor_msgs::Range>(
            range_attr);

    // PointCloud2 writer
    RoleAttributes pointcloud2_attr;
    pointcloud2_attr.set_channel_name("/sensor/pointcloud2");
    pointcloud2_attr.set_message_type(
        MessageType<autonomy::commsgs::proto::sensor_msgs::PointCloud2>());
    auto pointcloud2_writer =
        g_node
            ->CreateWriter<autonomy::commsgs::proto::sensor_msgs::PointCloud2>(
                pointcloud2_attr);

    // PointCloud writer
    RoleAttributes pointcloud_attr;
    pointcloud_attr.set_channel_name("/sensor/pointcloud");
    pointcloud_attr.set_message_type(
        MessageType<autonomy::commsgs::proto::sensor_msgs::PointCloud>());
    auto pointcloud_writer =
        g_node->CreateWriter<autonomy::commsgs::proto::sensor_msgs::PointCloud>(
            pointcloud_attr);

    // Path writer
    RoleAttributes path_attr;
    path_attr.set_channel_name("/planning/path");
    path_attr.set_message_type(
        MessageType<autonomy::commsgs::proto::planning_msgs::Path>());
    auto path_writer =
        g_node->CreateWriter<autonomy::commsgs::proto::planning_msgs::Path>(
            path_attr);

    // OccupancyGrid writer
    RoleAttributes map_attr;
    map_attr.set_channel_name("/map/occupancy_grid");
    map_attr.set_message_type(
        MessageType<autonomy::commsgs::proto::map_msgs::OccupancyGrid>());
    auto map_writer =
        g_node->CreateWriter<autonomy::commsgs::proto::map_msgs::OccupancyGrid>(
            map_attr);

    if (!imu_writer || !image_writer || !range_writer || !pointcloud2_writer ||
        !pointcloud_writer || !path_writer || !map_writer) {
        AERROR << "Failed to create one or more writers";
        return;
    }

    AINFO << "All writers created successfully. Waiting for topology "
             "discovery...";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 使用时间戳控制发布频率
    Time imu_last_time = Time::Now();
    Time image_last_time = Time::Now();
    Time range_last_time = Time::Now();
    Time pointcloud2_last_time = Time::Now();
    Time pointcloud_last_time = Time::Now();
    Time path_last_time = Time::Now();
    Time map_last_time = Time::Now();

    // 频率设置（Hz转换为纳秒间隔）
    constexpr int64_t IMU_INTERVAL_NS = 100000000LL;          // 10 Hz = 100ms
    constexpr int64_t IMAGE_INTERVAL_NS = 200000000LL;        // 5 Hz = 200ms
    constexpr int64_t RANGE_INTERVAL_NS = 50000000LL;         // 20 Hz = 50ms
    constexpr int64_t POINTCLOUD2_INTERVAL_NS = 500000000LL;  // 2 Hz = 500ms
    constexpr int64_t POINTCLOUD_INTERVAL_NS = 500000000LL;   // 2 Hz = 500ms
    constexpr int64_t PATH_INTERVAL_NS = 1000000000LL;        // 1 Hz = 1000ms
    constexpr int64_t MAP_INTERVAL_NS = 2000000000LL;         // 0.5 Hz = 2000ms

    uint64_t imu_seq = 0;
    uint64_t image_seq = 0;
    uint64_t range_seq = 0;
    uint64_t pointcloud2_seq = 0;
    uint64_t pointcloud_seq = 0;
    uint64_t path_seq = 0;
    uint64_t map_seq = 0;

    AINFO << "Starting to publish sensor data...";
    AINFO << "  IMU: 10 Hz";
    AINFO << "  Camera: 5 Hz";
    AINFO << "  Range: 20 Hz";
    AINFO << "  PointCloud2: 2 Hz";
    AINFO << "  PointCloud: 2 Hz";
    AINFO << "  Path: 1 Hz";
    AINFO << "  OccupancyGrid: 0.5 Hz";

    // 主循环
    Rate loop_rate(100.0);  // 主循环 100 Hz，用于检查所有传感器
    while (g_running && autolink::OK()) {
        Time now = Time::Now();

        // 发布 IMU (10 Hz)
        if ((now.ToNanosecond() - imu_last_time.ToNanosecond()) >=
            IMU_INTERVAL_NS) {
            PublishImu(imu_writer, imu_seq++);
            imu_last_time = now;
        }

        // 发布 Image (5 Hz)
        if ((now.ToNanosecond() - image_last_time.ToNanosecond()) >=
            IMAGE_INTERVAL_NS) {
            PublishImage(image_writer, image_seq++);
            image_last_time = now;
        }

        // 发布 Range (20 Hz)
        if ((now.ToNanosecond() - range_last_time.ToNanosecond()) >=
            RANGE_INTERVAL_NS) {
            PublishRange(range_writer, range_seq++);
            range_last_time = now;
        }

        // 发布 PointCloud2 (2 Hz)
        if ((now.ToNanosecond() - pointcloud2_last_time.ToNanosecond()) >=
            POINTCLOUD2_INTERVAL_NS) {
            PublishPointCloud2(pointcloud2_writer, pointcloud2_seq++);
            pointcloud2_last_time = now;
        }

        // 发布 PointCloud (2 Hz)
        if ((now.ToNanosecond() - pointcloud_last_time.ToNanosecond()) >=
            POINTCLOUD_INTERVAL_NS) {
            PublishPointCloud(pointcloud_writer, pointcloud_seq++);
            pointcloud_last_time = now;
        }

        // 发布 Path (1 Hz)
        if ((now.ToNanosecond() - path_last_time.ToNanosecond()) >=
            PATH_INTERVAL_NS) {
            PublishPath(path_writer, path_seq++);
            path_last_time = now;
        }

        // 发布 OccupancyGrid (0.5 Hz)
        if ((now.ToNanosecond() - map_last_time.ToNanosecond()) >=
            MAP_INTERVAL_NS) {
            PublishOccupancyGrid(map_writer, map_seq++);
            map_last_time = now;
        }

        // 控制主循环频率
        loop_rate.Sleep();
    }

    AINFO << "Data publisher stopped";
    autolink::WaitForShutdown();
}

}  // namespace
}  // namespace visualization
}  // namespace autonomy

int main(int argc, char** argv) {
    autolink::Init(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (autonomy::common::FLAGS_verbose) {
        autonomy::common::ShowVersion();
        return 0;
    }

    autonomy::visualization::Run();
    return 0;
}

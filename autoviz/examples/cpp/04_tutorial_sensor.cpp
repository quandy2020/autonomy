/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/point_field.pb.h>
#include <automsgs/msgs/sensor_msgs/range.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

constexpr int kRW = 160;
constexpr int kRH = 120;
constexpr int kDW = 80;
constexpr int kDH = 60;

using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;
using autoviz::examples::StampHeader;

std::shared_ptr<automsgs::msgs::sensor_msgs::Imu> MakeImu(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Imu>();
  StampHeader(msg->mutable_header());
  const double yaw = 0.3 * std::sin(t);
  msg->mutable_orientation()->set_z(std::sin(yaw * 0.5));
  msg->mutable_orientation()->set_w(std::cos(yaw * 0.5));
  msg->mutable_angular_velocity()->set_z(0.3 * std::cos(t));
  msg->mutable_linear_acceleration()->set_z(9.81);
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::Image> MakeRgb(int phase) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Image>();
  StampHeader(msg->mutable_header(), "camera");
  msg->set_height(kRH);
  msg->set_width(kRW);
  msg->set_encoding("rgb8");
  msg->set_step(kRW * 3);
  std::string buf(static_cast<size_t>(kRH * kRW * 3), '\0');
  for (int y = 0; y < kRH; ++y) {
    for (int x = 0; x < kRW; ++x) {
      const size_t i = static_cast<size_t>((y * kRW + x) * 3);
      buf[i] = static_cast<char>((x + phase) % 256);
      buf[i + 1] = static_cast<char>((y + phase) % 256);
      buf[i + 2] = static_cast<char>((x + y + phase) % 256);
    }
  }
  msg->set_data(std::move(buf));
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::Image> MakeDepth() {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Image>();
  StampHeader(msg->mutable_header(), "camera");
  msg->set_height(kDH);
  msg->set_width(kDW);
  msg->set_encoding("16UC1");
  msg->set_step(kDW * 2);
  std::string buf;
  buf.reserve(static_cast<size_t>(kDH * kDW * 2));
  const int cx = kDW / 2;
  const int cy = kDH / 2;
  for (int y = 0; y < kDH; ++y) {
    for (int x = 0; x < kDW; ++x) {
      const uint16_t mm =
          ((x - cx) * (x - cx) + (y - cy) * (y - cy) < 80) ? 1200 : 2000;
      char bytes[2];
      std::memcpy(bytes, &mm, 2);
      buf.append(bytes, 2);
    }
  }
  msg->set_data(std::move(buf));
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::CameraInfo> MakeCameraInfo() {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::CameraInfo>();
  StampHeader(msg->mutable_header(), "camera");
  msg->set_height(kDH);
  msg->set_width(kDW);
  msg->set_distortion_model("plumb_bob");
  const double fx = 60.0;
  const double fy = 60.0;
  const double cx = kDW / 2.0;
  const double cy = kDH / 2.0;
  for (int i = 0; i < 5; ++i) msg->add_d(0.0);
  for (double v : {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0}) msg->add_k(v);
  for (double v : {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}) msg->add_r(v);
  for (double v : {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0}) {
    msg->add_p(v);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::LaserScan> MakeScan(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::LaserScan>();
  StampHeader(msg->mutable_header(), "laser");
  constexpr int n = 180;
  msg->set_angle_min(-M_PI / 2);
  msg->set_angle_max(M_PI / 2);
  msg->set_angle_increment(M_PI / (n - 1));
  msg->set_range_min(0.1f);
  msg->set_range_max(10.0f);
  msg->set_scan_time(0.1f);
  for (int i = 0; i < n; ++i) {
    const double a = msg->angle_min() + i * msg->angle_increment();
    msg->add_ranges(static_cast<float>(2.0 + 0.5 * std::sin(a * 3 + t)));
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::PointCloud> MakePc(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::PointCloud>();
  StampHeader(msg->mutable_header());
  for (int i = 0; i < 40; ++i) {
    auto* p = msg->add_points();
    p->set_x(static_cast<float>(0.05 * i));
    p->set_y(static_cast<float>(std::sin(0.2 * i + t)));
    p->set_z(static_cast<float>(0.05 * (i % 5)));
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::PointCloud2> MakePc2(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::PointCloud2>();
  StampHeader(msg->mutable_header());
  constexpr int n = 80;
  msg->set_height(1);
  msg->set_width(n);
  msg->set_is_dense(true);
  msg->set_point_step(12);
  msg->set_row_step(12 * n);
  const char* names[] = {"x", "y", "z"};
  const uint32_t offs[] = {0, 4, 8};
  for (int i = 0; i < 3; ++i) {
    auto* f = msg->add_fields();
    f->set_name(names[i]);
    f->set_offset(offs[i]);
    f->set_count(1);
    f->set_datatype(automsgs::msgs::sensor_msgs::PointField::FLOAT32);
  }
  std::string buf(static_cast<size_t>(12 * n), '\0');
  for (int i = 0; i < n; ++i) {
    float xyz[3] = {static_cast<float>(0.05 * i),
                    static_cast<float>(std::sin(0.2 * i + t)),
                    static_cast<float>(0.05 * (i % 5))};
    std::memcpy(&buf[static_cast<size_t>(i * 12)], xyz, 12);
  }
  msg->set_data(std::move(buf));
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::Range> MakeRange(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Range>();
  StampHeader(msg->mutable_header(), "sonar");
  msg->set_radiation_type(automsgs::msgs::sensor_msgs::Range::ULTRASOUND);
  msg->set_field_of_view(0.3f);
  msg->set_min_range(0.05f);
  msg->set_max_range(4.0f);
  msg->set_range(static_cast<float>(1.5 + 0.5 * std::sin(t)));
  return msg;
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 10.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/sensor");
  auto w_imu = MakeWriter<automsgs::msgs::sensor_msgs::Imu>(node, "/fake/imu");
  auto w_rgb =
      MakeWriter<automsgs::msgs::sensor_msgs::Image>(node, "/fake/image");
  auto w_info = MakeWriter<automsgs::msgs::sensor_msgs::CameraInfo>(
      node, "/fake/camera_info");
  auto w_depth =
      MakeWriter<automsgs::msgs::sensor_msgs::Image>(node, "/fake/depth");
  auto w_scan = MakeWriter<automsgs::msgs::sensor_msgs::LaserScan>(
      node, "/fake/scan");
  auto w_pc = MakeWriter<automsgs::msgs::sensor_msgs::PointCloud>(
      node, "/fake/point_cloud");
  auto w_pc2 = MakeWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
      node, "/fake/point_cloud2");
  auto w_rng =
      MakeWriter<automsgs::msgs::sensor_msgs::Range>(node, "/fake/range");

  autolink::Rate rate(rate_hz);
  std::cout << "sensor @ " << rate_hz
            << " Hz → /fake/{imu,image,camera_info,depth,scan,point_cloud,"
               "point_cloud2,range}\n";

  const double t0 = NowSec();
  int phase = 0;
  while (autolink::OK()) {
    const double t = NowSec() - t0;
    w_imu->Write(MakeImu(t));
    w_rgb->Write(MakeRgb(phase));
    w_info->Write(MakeCameraInfo());
    w_depth->Write(MakeDepth());
    w_scan->Write(MakeScan(t));
    w_pc->Write(MakePc(t));
    w_pc2->Write(MakePc2(t));
    w_rng->Write(MakeRange(t));
    phase = (phase + 3) % 256;
    rate.Sleep();
  }
  return 0;
}

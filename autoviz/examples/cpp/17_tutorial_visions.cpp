/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/bounding_box2d_array.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>
#include <automsgs/msgs/vision_msgs/detection3d_array.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

constexpr int kW = 320;
constexpr int kH = 240;
constexpr const char* kClasses[] = {"person", "car", "bike", "dog"};

using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;
using autoviz::examples::StampHeader;

void SetBox2d(automsgs::msgs::vision_msgs::BoundingBox2D* box, double cx,
              double cy, double sx, double sy, double theta = 0.0) {
  box->mutable_center()->mutable_position()->set_x(cx);
  box->mutable_center()->mutable_position()->set_y(cy);
  box->mutable_center()->set_theta(theta);
  box->set_size_x(sx);
  box->set_size_y(sy);
}

void AddHyp(automsgs::msgs::vision_msgs::Detection2D* det,
            const std::string& class_id, double score) {
  auto* r = det->add_results();
  r->mutable_hypothesis()->set_class_id(class_id);
  r->mutable_hypothesis()->set_score(score);
}

void AddHyp3(automsgs::msgs::vision_msgs::Detection3D* det,
             const std::string& class_id, double score) {
  auto* r = det->add_results();
  r->mutable_hypothesis()->set_class_id(class_id);
  r->mutable_hypothesis()->set_score(score);
}

std::shared_ptr<automsgs::msgs::sensor_msgs::Image> MakeImage(int phase) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Image>();
  StampHeader(msg->mutable_header(), "camera");
  msg->set_height(kH);
  msg->set_width(kW);
  msg->set_encoding("rgb8");
  msg->set_step(kW * 3);
  std::string buf(static_cast<size_t>(kH * kW * 3), '\0');
  for (int y = 0; y < kH; ++y) {
    for (int x = 0; x < kW; ++x) {
      const size_t i = static_cast<size_t>((y * kW + x) * 3);
      buf[i] = static_cast<char>((40 + (x + phase) / 2) % 256);
      buf[i + 1] = static_cast<char>((60 + y / 2) % 256);
      buf[i + 2] = static_cast<char>(90);
    }
  }
  msg->set_data(std::move(buf));
  return msg;
}

std::shared_ptr<automsgs::msgs::vision_msgs::Detection2DArray>
MakeDetections2d(double t) {
  auto msg = std::make_shared<automsgs::msgs::vision_msgs::Detection2DArray>();
  StampHeader(msg->mutable_header(), "camera");
  struct Spec {
    double cx, cy, sx, sy;
    const char* cls;
    double score;
  };
  const Spec specs[] = {
      {80 + 40 * std::sin(t), 70, 60, 100, "person", 0.92},
      {200 + 30 * std::cos(t * 0.7), 140, 90, 50, "car", 0.81},
      {160, 50 + 20 * std::sin(t * 1.3), 40, 40, "bike", 0.66},
  };
  for (int i = 0; i < 3; ++i) {
    auto* det = msg->add_detections();
    StampHeader(det->mutable_header(), "camera");
    det->set_id(std::string(specs[i].cls) + "_" + std::to_string(i));
    SetBox2d(det->mutable_bbox(), specs[i].cx, specs[i].cy, specs[i].sx,
             specs[i].sy, 0.1 * std::sin(t + i));
    AddHyp(det, specs[i].cls, specs[i].score);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::vision_msgs::BoundingBox2DArray> MakeBoxes2d(
    double t) {
  auto msg =
      std::make_shared<automsgs::msgs::vision_msgs::BoundingBox2DArray>();
  StampHeader(msg->mutable_header(), "camera");
  for (int i = 0; i < 3; ++i) {
    const double a = t + i * 2.1;
    SetBox2d(msg->add_boxes(), 60 + 80 * i + 10 * std::sin(a),
             180 + 8 * std::cos(a), 50, 30, 0.05 * i);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::vision_msgs::Detection3DArray>
MakeDetections3d(double t) {
  auto msg = std::make_shared<automsgs::msgs::vision_msgs::Detection3DArray>();
  StampHeader(msg->mutable_header(), "map");
  for (int i = 0; i < 3; ++i) {
    const char* cls = kClasses[i];
    const double a = t * 0.5 + i * 2.0;
    auto* det = msg->add_detections();
    StampHeader(det->mutable_header(), "map");
    det->set_id(std::string(cls) + "_3d_" + std::to_string(i));
    auto* center = det->mutable_bbox()->mutable_center();
    center->mutable_position()->set_x(2.0 * std::cos(a));
    center->mutable_position()->set_y(2.0 * std::sin(a));
    center->mutable_position()->set_z(0.5 + 0.2 * i);
    center->mutable_orientation()->set_w(1.0);
    det->mutable_bbox()->mutable_size()->set_x(0.6 + 0.2 * i);
    det->mutable_bbox()->mutable_size()->set_y(0.4);
    det->mutable_bbox()->mutable_size()->set_z(
        0.5 + 0.3 * (std::string(cls) == "person"));
    AddHyp3(det, cls, 0.7 + 0.1 * i);
    auto* pose = det->mutable_results(0)->mutable_pose()->mutable_pose();
    StampHeader(pose->mutable_header(), "map");
    pose->mutable_pose()->CopyFrom(det->bbox().center());
  }
  return msg;
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 10.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/visions");
  auto w_img = MakeWriter<automsgs::msgs::sensor_msgs::Image>(
      node, "/fake/vision/image");
  auto w_d2 = MakeWriter<automsgs::msgs::vision_msgs::Detection2DArray>(
      node, "/fake/vision/detections2d");
  auto w_b2 = MakeWriter<automsgs::msgs::vision_msgs::BoundingBox2DArray>(
      node, "/fake/vision/boxes2d");
  auto w_d3 = MakeWriter<automsgs::msgs::vision_msgs::Detection3DArray>(
      node, "/fake/vision/detections3d");

  autolink::Rate rate(rate_hz);
  std::cout << "visions @ " << rate_hz
            << " Hz → /fake/vision/{image,detections2d,boxes2d,detections3d}\n";

  const double t0 = NowSec();
  int phase = 0;
  while (autolink::OK()) {
    const double t = NowSec() - t0;
    w_img->Write(MakeImage(phase));
    w_d2->Write(MakeDetections2d(t));
    w_b2->Write(MakeBoxes2d(t));
    w_d3->Write(MakeDetections3d(t));
    phase = (phase + 2) % 256;
    rate.Sleep();
  }
  return 0;
}

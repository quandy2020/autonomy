/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <iostream>
#include <memory>

#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;
using autoviz::examples::StampHeader;

void AddTf(automsgs::msgs::tf2_msgs::TFMessage* msg, const char* parent,
           const char* child, double x, double y, double z, double yaw = 0.0) {
  auto* ts = msg->add_transforms();
  StampHeader(ts->mutable_header(), parent);
  ts->set_child_frame_id(child);
  ts->mutable_transform()->mutable_translation()->set_x(x);
  ts->mutable_transform()->mutable_translation()->set_y(y);
  ts->mutable_transform()->mutable_translation()->set_z(z);
  ts->mutable_transform()->mutable_rotation()->set_z(std::sin(yaw * 0.5));
  ts->mutable_transform()->mutable_rotation()->set_w(std::cos(yaw * 0.5));
}

std::shared_ptr<automsgs::msgs::tf2_msgs::TFMessage> MakeTree(double t) {
  auto msg = std::make_shared<automsgs::msgs::tf2_msgs::TFMessage>();
  AddTf(msg.get(), "map", "base_link", 2.0 * std::cos(t), 2.0 * std::sin(t),
        0.0, t + M_PI / 2);
  AddTf(msg.get(), "base_link", "laser", 0.2, 0.0, 0.15);
  AddTf(msg.get(), "base_link", "camera", 0.1, 0.0, 0.3);
  return msg;
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 20.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/tf");
  auto w = MakeWriter<automsgs::msgs::tf2_msgs::TFMessage>(node, "/tf");

  autolink::Rate rate(rate_hz);
  std::cout << "tf @ " << rate_hz
            << " Hz → /tf (map→base_link→laser/camera)\n";

  const double t0 = NowSec();
  while (autolink::OK()) {
    w->Write(MakeTree(NowSec() - t0));
    rate.Sleep();
  }
  return 0;
}

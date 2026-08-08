/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <iostream>
#include <memory>
#include <string>

#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

constexpr int kW = 320;
constexpr int kH = 240;

using autoviz::examples::MakeWriter;
using autoviz::examples::ParseFlag;
using autoviz::examples::ParseRate;
using autoviz::examples::StampHeader;

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
      buf[i] = static_cast<char>((x + phase) % 256);
      buf[i + 1] = static_cast<char>((y + phase) % 256);
      buf[i + 2] = static_cast<char>((x + y + phase) % 256);
    }
  }
  msg->set_data(std::move(buf));
  return msg;
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 10.0);
  const bool is_static = ParseFlag(argc, argv, "--static");

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/image");
  auto w =
      MakeWriter<automsgs::msgs::sensor_msgs::Image>(node, "/fake/image");

  autolink::Rate rate(rate_hz);
  std::cout << "image @ " << rate_hz << " Hz → /fake/image (" << kW << "x"
            << kH << " rgb8)\n";

  int phase = 0;
  while (autolink::OK()) {
    w->Write(MakeImage(is_static ? 0 : phase));
    phase = (phase + 3) % 256;
    rate.Sleep();
  }
  return 0;
}

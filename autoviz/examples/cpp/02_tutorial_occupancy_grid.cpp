/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <iostream>
#include <memory>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

constexpr int kW = 40;
constexpr int kH = 30;
constexpr float kRes = 0.1f;

using autoviz::examples::MakeWriter;
using autoviz::examples::ParseRate;
using autoviz::examples::StampHeader;

std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid> MakeGrid() {
  auto msg = std::make_shared<automsgs::msgs::map_msgs::OccupancyGrid>();
  StampHeader(msg->mutable_header());
  auto* info = msg->mutable_info();
  info->set_resolution(kRes);
  info->set_width(kW);
  info->set_height(kH);
  info->mutable_origin()->mutable_position()->set_x(-0.5 * kW * kRes);
  info->mutable_origin()->mutable_position()->set_y(-0.5 * kH * kRes);
  info->mutable_origin()->mutable_orientation()->set_w(1.0);
  for (int y = 0; y < kH; ++y) {
    for (int x = 0; x < kW; ++x) {
      const bool edge = (x == 0 || x == kW - 1 || y == 0 || y == kH - 1);
      if (edge) {
        msg->add_data(100);
      } else if ((x + y) % 9 == 0) {
        msg->add_data(-1);
      } else {
        msg->add_data(0);
      }
    }
  }
  return msg;
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 1.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/grid");
  auto w = MakeWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
      node, "/fake/occupancy_grid");

  autolink::Rate rate(rate_hz);
  std::cout << "grid @ " << rate_hz << " Hz → /fake/occupancy_grid (" << kW
            << "x" << kH << ")\n";

  while (autolink::OK()) {
    w->Write(MakeGrid());
    rate.Sleep();
  }
  return 0;
}

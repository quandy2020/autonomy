/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <iostream>
#include <memory>

#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/std_msgs/float64.pb.h>

#include "common/tutorial_utils.hpp"

using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 50.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/plot");
  auto w_sin =
      MakeWriter<automsgs::msgs::std_msgs::Float64>(node, "/fake/plot/sine");
  auto w_cos =
      MakeWriter<automsgs::msgs::std_msgs::Float64>(node, "/fake/plot/cosine");
  auto w_cmd =
      MakeWriter<automsgs::msgs::geometry_msgs::Twist>(node, "/fake/plot/cmd");

  autolink::Rate rate(rate_hz);
  std::cout << "plot @ " << rate_hz
            << " Hz → /fake/plot/{sine,cosine,cmd}\n";

  const double t0 = NowSec();
  while (autolink::OK()) {
    const double t = NowSec() - t0;

    auto sin_msg = std::make_shared<automsgs::msgs::std_msgs::Float64>();
    sin_msg->set_data(std::sin(t));
    w_sin->Write(sin_msg);

    auto cos_msg = std::make_shared<automsgs::msgs::std_msgs::Float64>();
    cos_msg->set_data(std::cos(t * 1.5));
    w_cos->Write(cos_msg);

    auto cmd = std::make_shared<automsgs::msgs::geometry_msgs::Twist>();
    cmd->mutable_linear()->set_x(0.5 + 0.5 * std::sin(t * 0.7));
    cmd->mutable_angular()->set_z(0.3 * std::cos(t));
    w_cmd->Write(cmd);

    rate.Sleep();
  }
  return 0;
}

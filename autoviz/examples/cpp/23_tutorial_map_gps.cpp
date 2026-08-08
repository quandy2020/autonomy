/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <iostream>
#include <memory>

#include <automsgs/msgs/sensor_msgs/nav_sat_fix.pb.h>
#include <automsgs/msgs/sensor_msgs/nav_sat_status.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

constexpr double kLat0 = 31.2304;
constexpr double kLon0 = 121.4737;
constexpr double kDLat = 0.001;
const double kDLon = 0.001 / std::cos(kLat0 * M_PI / 180.0);

using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;
using autoviz::examples::StampHeader;

std::shared_ptr<automsgs::msgs::sensor_msgs::NavSatFix> MakeFix(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::NavSatFix>();
  StampHeader(msg->mutable_header(), "gps");
  msg->mutable_status()->set_status(
      automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_FIX);
  msg->mutable_status()->set_service(
      automsgs::msgs::sensor_msgs::NavSatStatus::SERVICE_GPS);
  const double a = t * 0.15;
  msg->set_latitude(kLat0 + kDLat * std::sin(a));
  msg->set_longitude(kLon0 + kDLon * std::cos(a));
  msg->set_altitude(12.0 + std::sin(a));
  for (double v : {4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 16.0}) {
    msg->add_position_covariance(v);
  }
  msg->set_position_covariance_type(
      automsgs::msgs::sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
  return msg;
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 5.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/map_gps");
  auto w = MakeWriter<automsgs::msgs::sensor_msgs::NavSatFix>(node,
                                                             "/gps/fix");

  autolink::Rate rate(rate_hz);
  std::cout << "map_gps @ " << rate_hz << " Hz → /gps/fix\n";

  const double t0 = NowSec();
  while (autolink::OK()) {
    w->Write(MakeFix(NowSec() - t0));
    rate.Sleep();
  }
  return 0;
}

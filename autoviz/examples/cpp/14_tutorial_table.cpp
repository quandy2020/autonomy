/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <automsgs/msgs/diagnostic_msgs/diagnostic_array.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_array.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

using DS = automsgs::msgs::diagnostic_msgs::DiagnosticStatus;
using autoviz::examples::SetYawPose;
using autoviz::examples::StampHeader;

void AddKv(DS* st, const std::string& key, const std::string& value) {
  auto* kv = st->add_values();
  kv->set_key(key);
  kv->set_value(value);
}

std::shared_ptr<automsgs::msgs::diagnostic_msgs::DiagnosticArray> MakeStatus(
    double t) {
  auto msg =
      std::make_shared<automsgs::msgs::diagnostic_msgs::DiagnosticArray>();
  StampHeader(msg->mutable_header());

  auto add = [&](DS::Level level, const char* name, const char* message,
                 const char* hw, const std::string& val) {
    auto* st = msg->add_status();
    st->set_level(level);
    st->set_name(name);
    st->set_message(message);
    st->set_hardware_id(hw);
    AddKv(st, "value", val);
  };

  {
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(2);
    ss << (12.4 + 0.1 * std::sin(t)) << " V";
    add(DS::OK, "battery", "nominal", "batt0", ss.str());
  }
  {
    std::ostringstream ss;
    ss << static_cast<int>(65 + 10 * std::sin(t * 0.7)) << " C";
    add(DS::WARN, "cpu", "hot", "cpu0", ss.str());
  }
  {
    std::ostringstream ss;
    ss << static_cast<int>(20 + 5 * std::sin(t)) << " Hz";
    add(DS::OK, "lidar", "streaming", "lidar0", ss.str());
  }
  {
    const auto lvl = std::sin(t) > 0.8 ? DS::ERROR : DS::OK;
    std::ostringstream ss;
    ss << (4 + static_cast<int>(2 * std::abs(std::sin(t)))) << " sats";
    add(lvl, "gps", "fix", "gps0", ss.str());
  }
  add(DS::STALE, "camera", "no frame", "cam0", "0 Hz");
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::PoseArray> MakePoses(double t) {
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::PoseArray>();
  StampHeader(msg->mutable_header());
  for (int i = 0; i < 8; ++i) {
    const double a = t + i * (2.0 * M_PI / 8.0);
    SetYawPose(msg->add_poses(), 2.0 * std::cos(a), 2.0 * std::sin(a), a,
               0.05 * i);
  }
  return msg;
}

}  // namespace

using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 5.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/table");
  auto w_st =
      MakeWriter<automsgs::msgs::diagnostic_msgs::DiagnosticArray>(
          node, "/fake/table/status");
  auto w_poses = MakeWriter<automsgs::msgs::geometry_msgs::PoseArray>(
      node, "/fake/table/poses");

  autolink::Rate rate(rate_hz);
  std::cout << "table @ " << rate_hz
            << " Hz → /fake/table/{status,poses}\n";

  const double t0 = NowSec();
  while (autolink::OK()) {
    const double t = NowSec() - t0;
    w_st->Write(MakeStatus(t));
    w_poses->Write(MakePoses(t));
    rate.Sleep();
  }
  return 0;
}

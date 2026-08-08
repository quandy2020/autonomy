/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <automsgs/msgs/diagnostic_msgs/diagnostic_array.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

using DS = automsgs::msgs::diagnostic_msgs::DiagnosticStatus;
using autoviz::examples::StampHeader;

void Kv(DS* st, const std::string& key, const std::string& value) {
  auto* item = st->add_values();
  item->set_key(key);
  item->set_value(value);
}

void Add(automsgs::msgs::diagnostic_msgs::DiagnosticArray* msg, DS::Level level,
         const std::string& name, const std::string& message,
         const std::string& hw,
         const std::vector<std::pair<std::string, std::string>>& pairs) {
  auto* st = msg->add_status();
  st->set_level(level);
  st->set_name(name);
  st->set_message(message);
  st->set_hardware_id(hw);
  for (const auto& p : pairs) Kv(st, p.first, p.second);
}

std::string F2(double v) {
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(2);
  ss << v;
  return ss.str();
}

std::string F3(double v) {
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(3);
  ss << v;
  return ss.str();
}

std::string F1(double v) {
  std::ostringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(1);
  ss << v;
  return ss.str();
}

std::shared_ptr<automsgs::msgs::diagnostic_msgs::DiagnosticArray>
MakeDiagnostics(double t) {
  auto msg =
      std::make_shared<automsgs::msgs::diagnostic_msgs::DiagnosticArray>();
  StampHeader(msg->mutable_header(), "");

  const double batt = 12.6 + 0.4 * std::sin(t * 0.3);
  const auto batt_lvl =
      batt > 12.0 ? DS::OK : (batt > 11.5 ? DS::WARN : DS::ERROR);
  Add(msg.get(), batt_lvl, "power:battery",
      batt_lvl == DS::OK ? "nominal" : "low voltage", "batt0",
      {{"voltage", F2(batt)},
       {"current", F2(2.1 + 0.5 * std::sin(t))},
       {"soc", std::to_string(static_cast<int>(70 + 20 * std::sin(t * 0.2))) +
                   "%"}});

  const double cpu = 55 + 35 * (0.5 + 0.5 * std::sin(t * 0.8));
  const auto cpu_lvl = cpu < 75 ? DS::OK : (cpu < 90 ? DS::WARN : DS::ERROR);
  Add(msg.get(), cpu_lvl, "system:cpu",
      cpu_lvl == DS::OK ? "ok" : "thermal", "cpu0",
      {{"load", std::to_string(static_cast<int>(cpu)) + "%"},
       {"temp_c", std::to_string(static_cast<int>(40 + cpu * 0.3))}});

  const double hz = 20 + 5 * std::sin(t);
  const int drop = std::max(0, static_cast<int>(3 * std::sin(t * 1.3)));
  const auto lidar_lvl =
      drop == 0 ? DS::OK : (drop < 3 ? DS::WARN : DS::ERROR);
  Add(msg.get(), lidar_lvl, "sensor:lidar",
      drop == 0 ? "streaming" : ("dropout x" + std::to_string(drop)), "lidar0",
      {{"rate_hz", F1(hz)},
       {"points",
        std::to_string(static_cast<int>(18000 + 1000 * std::sin(t)))},
       {"drop_frames", std::to_string(drop)}});

  const int sats = 4 + static_cast<int>(3 * std::abs(std::sin(t * 0.5)));
  const bool fix = sats >= 5;
  const auto gps_lvl = fix ? DS::OK : DS::WARN;
  Add(msg.get(), gps_lvl, "sensor:gps", fix ? "3d fix" : "no fix", "gps0",
      {{"satellites", std::to_string(sats)},
       {"hdop", F2(1.2 + 0.8 * (fix ? 0 : 1))}});

  const auto cam_lvl =
      (static_cast<int>(t) % 12 >= 9) ? DS::STALE : DS::OK;
  Add(msg.get(), cam_lvl, "sensor:camera",
      cam_lvl == DS::STALE ? "no frame" : "ok", "cam0",
      {{"fps", cam_lvl == DS::STALE ? "0" : "30"}, {"exposure_ms", "8"}});

  const bool ctrl_ok = std::sin(t * 0.6) > -0.7;
  Add(msg.get(), ctrl_ok ? DS::OK : DS::ERROR, "nav:controller",
      ctrl_ok ? "tracking" : "cmd_vel timeout", "ctrl0",
      {{"tracking_err_m", F3(0.03 + 0.2 * (!ctrl_ok))},
       {"mode", ctrl_ok ? "AUTO" : "HOLD"}});

  return msg;
}

}  // namespace

using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 2.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/diagnostics");
  auto writer =
      MakeWriter<automsgs::msgs::diagnostic_msgs::DiagnosticArray>(
          node, "/diagnostics");

  autolink::Rate rate(rate_hz);
  std::cout << "diagnostics @ " << rate_hz << " Hz → /diagnostics\n";

  const double t0 = NowSec();
  while (autolink::OK()) {
    writer->Write(MakeDiagnostics(NowSec() - t0));
    rate.Sleep();
  }
  return 0;
}

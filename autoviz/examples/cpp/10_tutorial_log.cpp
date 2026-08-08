/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/tutorial_utils.hpp"

namespace {

constexpr int kDebug = 1;
constexpr int kInfo = 2;
constexpr int kWarn = 3;
constexpr int kError = 4;

struct Entry {
  int level;
  const char* name;
  const char* message;
};

const Entry kEntries[] = {
    {kDebug, "sensor", "imu sample ok"},
    {kInfo, "planner", "path length=12.4 m"},
    {kInfo, "controller", "tracking error=0.03 m"},
    {kWarn, "sensor", "lidar dropout 2 frames"},
    {kWarn, "planner", "goal near obstacle"},
    {kError, "controller", "cmd_vel timeout"},
    {kError, "autolink", "channel /cmd_vel no reader"},
};

std::string MakeLogJson(int level, const std::string& name,
                        const std::string& message, int line) {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  const int64_t sec = ns / 1000000000LL;
  const int64_t nsec = ns % 1000000000LL;
  return std::string("{\"timestamp\":{\"sec\":") + std::to_string(sec) +
         ",\"nsec\":" + std::to_string(nsec) + "},\"level\":" +
         std::to_string(level) + ",\"message\":\"" + message +
         "\",\"name\":\"" + name +
         "\",\"file\":\"10_tutorial_log.cpp\",\"line\":" +
         std::to_string(line) + "}";
}

}  // namespace

using autoviz::examples::MakeRawWriter;
using autoviz::examples::ParseRate;

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 2.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/log");
  auto writer = MakeRawWriter(node, "/fake/log", "foxglove.Log");

  autolink::Rate rate(rate_hz);
  std::cout << "log @ " << rate_hz << " Hz → /fake/log (foxglove.Log JSON)\n";

  constexpr int kN = static_cast<int>(sizeof(kEntries) / sizeof(kEntries[0]));
  int i = 0;
  while (autolink::OK()) {
    const Entry& e = kEntries[i % kN];
    const std::string msg =
        std::string(e.message) + " #" + std::to_string(i);
    const std::string json =
        MakeLogJson(e.level, e.name, msg, 40 + (i % 7));
    auto raw = std::make_shared<autolink::message::RawMessage>(json);
    writer->Write(raw);
    ++i;
    rate.Sleep();
  }
  return 0;
}

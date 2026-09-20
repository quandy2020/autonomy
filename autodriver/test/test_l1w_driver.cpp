/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file test_l1w_driver.cpp
 * @brief L1-W HighLevel tests: simulate (default) + optional live SDK/HW.
 *
 * Hardware suite (opt-in, never run in CI by default):
 *   AUTODRIVER_L1W_HW_TEST=1
 *   AUTODRIVER_L1W_HOST=192.168.168.168          # dog IP
 *   AUTODRIVER_L1W_LOCAL_IP=192.168.168.168     # onboard eth0 / AGX IP
 *   AUTODRIVER_L1W_LOCAL_PORT=43988              # optional
 *   AUTODRIVER_L1W_HW_MOTION=1                  # optional tiny move/crawl/velocity
 *   AUTODRIVER_L1W_HW_SPECIAL=1                 # optional shake/squat/climb
 *
 * Requires libautodriver_l1w built with GenisomL1w SDK linked.
 */

#include "chassis/l1w/driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>

#include <gtest/gtest.h>

#include "chassis/backend_registry.hpp"
#include "chassis/operational_mode.hpp"
#include "chassis/tool_command.hpp"
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>

namespace {

const char* EnvOr(const char* key, const char* fallback) {
  const char* value = std::getenv(key);
  if (value == nullptr || value[0] == '\0') {
    return fallback;
  }
  return value;
}

bool EnvTruthy(const char* key) {
  const char* value = std::getenv(key);
  if (value == nullptr || value[0] == '\0') {
    return false;
  }
  return std::strcmp(value, "1") == 0 || std::strcmp(value, "true") == 0 ||
         std::strcmp(value, "TRUE") == 0 || std::strcmp(value, "yes") == 0 ||
         std::strcmp(value, "YES") == 0 || std::strcmp(value, "on") == 0;
}

void SleepMs(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/** @brief Pause between live HW actions so the dog can finish each step. */
constexpr int kHwActionGapMs = 2000;

/** @brief Open-loop travel distance for VelocityCommandAxes (meters). */
constexpr double kHwTravelMeters = 1.0;

/** @brief Body-frame speeds within Genisom valid bands (|v| >= 0.1). */
constexpr double kHwVxMps = 0.2;
constexpr double kHwVyMps = 0.1;

int TravelDurationMs(double speed_mps, double meters) {
  if (speed_mps < 1e-6) {
    return 0;
  }
  return static_cast<int>((meters / speed_mps) * 1000.0 + 0.5);
}

bool ApplyBodyTwist(autodriver::chassis::ChassisDriver& driver, double vx,
                    double vy, double wz) {
  autodriver::chassis::ChassisCommand cmd;
  if (std::abs(vx) > 1e-9 || std::abs(vy) > 1e-9 || std::abs(wz) > 1e-9) {
    cmd.mutable_twist()->mutable_linear()->set_x(vx);
    cmd.mutable_twist()->mutable_linear()->set_y(vy);
    cmd.mutable_twist()->mutable_angular()->set_z(wz);
  }
  return driver.ApplyVelocityCommand(cmd);
}

bool StopBodyTwist(autodriver::chassis::ChassisDriver& driver) {
  // Hold with zero twist only — do not standUp here (causes squat after each axis).
  (void)ApplyBodyTwist(driver, 0.0, 0.0, 0.0);
  // Allow Genisom to leave 'move' so the next standUp is accepted.
  SleepMs(kHwActionGapMs);
  return true;
}

bool EnsureStanding(autodriver::chassis::ChassisDriver& driver) {
  autodriver::chassis::ToolCommand stand;
  stand.name = "stand";
  if (!driver.ApplyToolCommand(stand)) {
    // Still in move: stop, wait, retry stand once.
    (void)ApplyBodyTwist(driver, 0.0, 0.0, 0.0);
    SleepMs(kHwActionGapMs);
    if (!driver.ApplyToolCommand(stand)) {
      return false;
    }
  }
  return true;
}

bool DriveAxisForMeters(autodriver::chassis::ChassisDriver& driver, double vx,
                        double vy, double meters) {
  // standUp before each axis (after previous zero-twist stop), not after.
  if (!EnsureStanding(driver)) {
    return false;
  }
  const double speed = std::max(std::abs(vx), std::abs(vy));
  const int total_ms = TravelDurationMs(speed, meters);
  constexpr int kPulseMs = 400;
  int elapsed = 0;
  int failures = 0;
  while (elapsed < total_ms) {
    if (!ApplyBodyTwist(driver, vx, vy, 0.0)) {
      ++failures;
      (void)ApplyBodyTwist(driver, 0.0, 0.0, 0.0);
      SleepMs(kHwActionGapMs);
      if (!EnsureStanding(driver) || !ApplyBodyTwist(driver, vx, vy, 0.0)) {
        (void)StopBodyTwist(driver);
        return false;
      }
    }
    const int step = std::min(kPulseMs, total_ms - elapsed);
    SleepMs(step);
    elapsed += step;
  }
  if (failures > 0) {
    std::cerr << "[L1wDriverHw] recovered from " << failures
              << " move pulse failure(s)\n";
  }
  return StopBodyTwist(driver);
}

bool LocalIpv4Assigned(const std::string& ip) {
  ifaddrs* head = nullptr;
  if (getifaddrs(&head) != 0) {
    return false;
  }
  bool found = false;
  for (ifaddrs* it = head; it != nullptr; it = it->ifa_next) {
    if (it->ifa_addr == nullptr || it->ifa_addr->sa_family != AF_INET) {
      continue;
    }
    char buf[INET_ADDRSTRLEN] = {};
    const auto* sin = reinterpret_cast<sockaddr_in*>(it->ifa_addr);
    if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)) == nullptr) {
      continue;
    }
    if (ip == buf) {
      found = true;
      break;
    }
  }
  freeifaddrs(head);
  return found;
}

std::string PreferLocalIpv4(const char* preferred,
                            const char* fallback_host) {
  if (preferred != nullptr && preferred[0] != '\0' &&
      LocalIpv4Assigned(preferred)) {
    return preferred;
  }
  if (fallback_host != nullptr && fallback_host[0] != '\0' &&
      LocalIpv4Assigned(fallback_host)) {
    if (preferred != nullptr && preferred[0] != '\0' &&
        preferred != std::string(fallback_host)) {
      std::cerr << "[L1wDriverHw] AUTODRIVER_L1W_LOCAL_IP=" << preferred
                << " is not assigned on this host; using " << fallback_host
                << " (onboard)\n";
    }
    return fallback_host;
  }
  // Prefer eth0-style dog LAN, then any non-loopback IPv4.
  constexpr const char* kPrefer[] = {"192.168.168.168", "192.168.234.1"};
  for (const char* cand : kPrefer) {
    if (LocalIpv4Assigned(cand)) {
      if (preferred != nullptr && preferred[0] != '\0') {
        std::cerr << "[L1wDriverHw] AUTODRIVER_L1W_LOCAL_IP=" << preferred
                  << " is not assigned; using " << cand << "\n";
      }
      return cand;
    }
  }
  ifaddrs* head = nullptr;
  if (getifaddrs(&head) == 0) {
    std::string picked;
    for (ifaddrs* it = head; it != nullptr; it = it->ifa_next) {
      if (it->ifa_addr == nullptr || it->ifa_addr->sa_family != AF_INET) {
        continue;
      }
      char buf[INET_ADDRSTRLEN] = {};
      const auto* sin = reinterpret_cast<sockaddr_in*>(it->ifa_addr);
      if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)) == nullptr) {
        continue;
      }
      if (std::strcmp(buf, "127.0.0.1") == 0) {
        continue;
      }
      picked = buf;
      break;
    }
    freeifaddrs(head);
    if (!picked.empty()) {
      return picked;
    }
  }
  return preferred != nullptr && preferred[0] != '\0' ? preferred
                                                      : "192.168.168.168";
}

autodriver::chassis::ChassisDriver::SharedPtr MakeSimDriver() {
  autodriver::hardware::DriverParams params;
  params["simulate"] = "true";
  params["auto_stand"] = "true";
  params["allow_lateral"] = "true";
  params["max_vx"] = "1.0";
  params["max_vy"] = "0.5";
  params["max_wz"] = "1.0";
  return autodriver::chassis::ChassisDriver::SharedPtr(
      autodriver::chassis::CreateL1wChassisDriver("chassis/l1w", params));
}

autodriver::chassis::ChassisDriver::SharedPtr MakeHwDriver() {
  autodriver::hardware::DriverParams params;
  params["simulate"] = "false";
  // Onboard Firefly: host == local_ip == eth0. Remote AGX: set LOCAL_IP to the
  // AGX address and match /opt/export/config/sdk_config.yaml target_ip.
  const std::string host = EnvOr("AUTODRIVER_L1W_HOST", "192.168.168.168");
  const std::string local_ip = PreferLocalIpv4(
      EnvOr("AUTODRIVER_L1W_LOCAL_IP", "192.168.168.168"), host.c_str());
  params["host"] = host;
  params["local_ip"] = local_ip;
  params["local_port"] = EnvOr("AUTODRIVER_L1W_LOCAL_PORT", "43988");
  params["auto_stand"] = EnvOr("AUTODRIVER_L1W_AUTO_STAND", "true");
  // Keep standing after Stop() so motion suites do not lie/squat between cases.
  params["lie_on_stop"] = EnvOr("AUTODRIVER_L1W_LIE_ON_STOP", "false");
  params["allow_lateral"] = "true";
  params["sample_joints"] = "true";
  params["default_gait"] = "move";
  params["connect_timeout_ms"] = EnvOr("AUTODRIVER_L1W_CONNECT_TIMEOUT_MS", "3000");
  params["stand_settle_ms"] = EnvOr("AUTODRIVER_L1W_STAND_SETTLE_MS", "2000");
  params["max_vx"] = "0.2";
  params["max_vy"] = "0.1";
  params["max_wz"] = "0.3";
  return autodriver::chassis::ChassisDriver::SharedPtr(
      autodriver::chassis::CreateL1wChassisDriver("chassis/l1w_hw", params));
}

void SkipUnlessHwReady() {
  if (!EnvTruthy("AUTODRIVER_L1W_HW_TEST")) {
    GTEST_SKIP() << "set AUTODRIVER_L1W_HW_TEST=1 for live L1-W SDK tests";
  }
#if !defined(AUTODRIVER_HAVE_GENISOM_L1W)
  GTEST_SKIP() << "autodriver_l1w built without GenisomL1w SDK "
                  "(install thirdparty/zsl1w or -DGenisomL1w_ROOT=)";
#endif
}

TEST(L1wDriver, WheelMoveWithLateral) {
  auto driver = MakeSimDriver();
  ASSERT_TRUE(driver->Start());
  EXPECT_TRUE(driver->ApplyLocomotionIntent(
      autodriver::chassis::LocomotionIntent::kWheel));

  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.2);
  cmd.mutable_twist()->mutable_linear()->set_y(0.1);
  cmd.mutable_twist()->mutable_angular()->set_z(0.05);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));

  autodriver::chassis::ChassisState state;
  EXPECT_TRUE(driver->ReadChassisState(&state));
  EXPECT_NEAR(state.twist().twist().linear().y(), 0.1, 1e-6);
  driver->Stop();
}

TEST(L1wDriver, WalkUsesCrawlGaitAndTools) {
  auto driver = MakeSimDriver();
  ASSERT_TRUE(driver->Start());
  EXPECT_TRUE(driver->ApplyLocomotionIntent(
      autodriver::chassis::LocomotionIntent::kWalk));

  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.15);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));

  autodriver::chassis::ToolCommand cancel;
  cancel.name = "cancel_crawl";
  EXPECT_TRUE(driver->ApplyToolCommand(cancel));

  autodriver::chassis::ToolCommand attitude;
  attitude.name = "attitude";
  attitude.value = "0.1,-0.1,0,0.05";
  EXPECT_TRUE(driver->ApplyToolCommand(attitude));

  autodriver::chassis::ToolCommand lie;
  lie.name = "lie";
  EXPECT_TRUE(driver->ApplyToolCommand(lie));

  autodriver::chassis::ToolCommand stand;
  stand.name = "stand";
  EXPECT_TRUE(driver->ApplyToolCommand(stand));

  autodriver::chassis::ToolCommand passive;
  passive.name = "passive";
  EXPECT_TRUE(driver->ApplyToolCommand(passive));

  EXPECT_TRUE(driver->TriggerEmergencyStop());
  driver->Stop();
}

TEST(L1wDriver, ClimbShakeAndRearSquat) {
  auto driver = MakeSimDriver();
  ASSERT_TRUE(driver->Start());

  // Climb is L1-W-only (tool / default_gait), not a shared LocomotionIntent.
  autodriver::chassis::ToolCommand climb;
  climb.name = "climb";
  EXPECT_TRUE(driver->ApplyToolCommand(climb));

  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.1);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));

  autodriver::chassis::ToolCommand cancel_climb;
  cancel_climb.name = "cancel_climb";
  EXPECT_TRUE(driver->ApplyToolCommand(cancel_climb));

  autodriver::chassis::ToolCommand shake;
  shake.name = "shake_hand";
  EXPECT_TRUE(driver->ApplyToolCommand(shake));

  autodriver::chassis::ToolCommand squat;
  squat.name = "rear_squat";
  EXPECT_TRUE(driver->ApplyToolCommand(squat));

  autodriver::chassis::ToolCommand climb_once;
  climb_once.name = "climb";
  climb_once.value = "0.05,0,0";
  EXPECT_TRUE(driver->ApplyToolCommand(climb_once));

  driver->Stop();
}

TEST(L1wDriver, StandIgnoresNonZeroTwist) {
  auto driver = MakeSimDriver();
  ASSERT_TRUE(driver->Start());
  EXPECT_TRUE(driver->ApplyLocomotionIntent(
      autodriver::chassis::LocomotionIntent::kStand));
  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.3);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));
  autodriver::chassis::ChassisState state;
  ASSERT_TRUE(driver->ReadChassisState(&state));
  EXPECT_NEAR(state.twist().twist().linear().x(), 0.0, 1e-6);
  driver->Stop();
}

TEST(L1wDriver, VelocityCommandClampAndAxes) {
  autodriver::hardware::DriverParams params;
  params["simulate"] = "true";
  params["auto_stand"] = "true";
  params["allow_lateral"] = "true";
  params["max_vx"] = "0.2";
  params["max_vy"] = "0.1";
  params["max_wz"] = "0.3";
  auto driver = autodriver::chassis::ChassisDriver::SharedPtr(
      autodriver::chassis::CreateL1wChassisDriver("chassis/l1w_vel", params));
  ASSERT_TRUE(driver->Start());
  ASSERT_TRUE(driver->ApplyLocomotionIntent(
      autodriver::chassis::LocomotionIntent::kWheel));

  autodriver::chassis::ChassisCommand cmd;
  autodriver::chassis::ChassisState state;

  // Over-limit inputs are clamped to max_*.
  cmd.mutable_twist()->mutable_linear()->set_x(1.0);
  cmd.mutable_twist()->mutable_linear()->set_y(0.5);
  cmd.mutable_twist()->mutable_angular()->set_z(-1.0);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));
  ASSERT_TRUE(driver->ReadChassisState(&state));
  EXPECT_NEAR(state.twist().twist().linear().x(), 0.2, 1e-6);
  EXPECT_NEAR(state.twist().twist().linear().y(), 0.1, 1e-6);
  EXPECT_NEAR(state.twist().twist().angular().z(), -0.3, 1e-6);

  // Forward / lateral / yaw axes independently.
  cmd.Clear();
  cmd.mutable_twist()->mutable_linear()->set_x(0.15);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));
  ASSERT_TRUE(driver->ReadChassisState(&state));
  EXPECT_NEAR(state.twist().twist().linear().x(), 0.15, 1e-6);
  EXPECT_NEAR(state.twist().twist().linear().y(), 0.0, 1e-6);
  EXPECT_NEAR(state.twist().twist().angular().z(), 0.0, 1e-6);

  cmd.Clear();
  cmd.mutable_twist()->mutable_linear()->set_y(-0.08);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));
  ASSERT_TRUE(driver->ReadChassisState(&state));
  EXPECT_NEAR(state.twist().twist().linear().x(), 0.0, 1e-6);
  EXPECT_NEAR(state.twist().twist().linear().y(), -0.08, 1e-6);

  cmd.Clear();
  cmd.mutable_twist()->mutable_angular()->set_z(0.2);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));
  ASSERT_TRUE(driver->ReadChassisState(&state));
  EXPECT_NEAR(state.twist().twist().angular().z(), 0.2, 1e-6);

  // Empty / zero twist stops motion.
  cmd.Clear();
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));
  ASSERT_TRUE(driver->ReadChassisState(&state));
  EXPECT_NEAR(state.twist().twist().linear().x(), 0.0, 1e-6);
  EXPECT_NEAR(state.twist().twist().linear().y(), 0.0, 1e-6);
  EXPECT_NEAR(state.twist().twist().angular().z(), 0.0, 1e-6);

  driver->Stop();
}

TEST(L1wDriver, VelocityCommandDisablesLateral) {
  autodriver::hardware::DriverParams params;
  params["simulate"] = "true";
  params["auto_stand"] = "true";
  params["allow_lateral"] = "false";
  params["max_vx"] = "0.5";
  params["max_vy"] = "0.5";
  params["max_wz"] = "0.5";
  auto driver = autodriver::chassis::ChassisDriver::SharedPtr(
      autodriver::chassis::CreateL1wChassisDriver("chassis/l1w_no_vy", params));
  ASSERT_TRUE(driver->Start());
  ASSERT_TRUE(driver->ApplyLocomotionIntent(
      autodriver::chassis::LocomotionIntent::kWheel));

  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.1);
  cmd.mutable_twist()->mutable_linear()->set_y(0.2);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));

  autodriver::chassis::ChassisState state;
  ASSERT_TRUE(driver->ReadChassisState(&state));
  EXPECT_NEAR(state.twist().twist().linear().x(), 0.1, 1e-6);
  EXPECT_NEAR(state.twist().twist().linear().y(), 0.0, 1e-6);
  driver->Stop();
}

TEST(L1wBackendRegistry, Aliases) {
  autodriver::hardware::DriverParams params;
  params["simulate"] = "true";
  auto warmup = std::unique_ptr<autodriver::chassis::ChassisDriver>(
      autodriver::chassis::CreateL1wChassisDriver("chassis/warmup", params));
  ASSERT_NE(warmup, nullptr);

  for (const char* name : {"l1w", "genisom", "zsibot", "zsl-1w", "l1-w"}) {
    auto driver =
        autodriver::chassis::ChassisBackendRegistry::Instance().CreateDriver(
            name, "chassis/base", params);
    ASSERT_NE(driver, nullptr) << name;
    EXPECT_TRUE(driver->Start()) << name;
    driver->Stop();
  }
}

// ---------------------------------------------------------------------------
// Live SDK / hardware (opt-in)
// ---------------------------------------------------------------------------

TEST(L1wDriverHw, ConnectAndSampleState) {
  SkipUnlessHwReady();

  auto driver = MakeHwDriver();
  ASSERT_TRUE(driver->Start())
      << "SDK checkConnect/standUp failed; check host="
      << EnvOr("AUTODRIVER_L1W_HOST", "192.168.168.168")
      << " local_ip=" << EnvOr("AUTODRIVER_L1W_LOCAL_IP", "192.168.168.168")
      << " and dog sdk_config.target_ip";

  SleepMs(kHwActionGapMs);

  autodriver::chassis::ChassisState state;
  ASSERT_TRUE(driver->ReadChassisState(&state));
  EXPECT_GE(state.battery_percent(), 0.0f);
  EXPECT_LE(state.battery_percent(), 100.0f);
  EXPECT_FALSE(state.map_name().empty());
  EXPECT_NE(state.map_name().find("ctrl:"), std::string::npos);

  driver->Stop();
}

TEST(L1wDriverHw, WheelMoveStop) {
  SkipUnlessHwReady();
  if (!EnvTruthy("AUTODRIVER_L1W_HW_MOTION")) {
    GTEST_SKIP() << "set AUTODRIVER_L1W_HW_MOTION=1 to enable tiny move";
  }

  auto driver = MakeHwDriver();
  ASSERT_TRUE(driver->Start());
  SleepMs(kHwActionGapMs);
  ASSERT_TRUE(driver->ApplyLocomotionIntent(
      autodriver::chassis::LocomotionIntent::kWheel));
  SleepMs(kHwActionGapMs);

  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.05);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));
  SleepMs(kHwActionGapMs);

  cmd.Clear();
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));
  SleepMs(kHwActionGapMs);

  autodriver::chassis::ChassisState state;
  EXPECT_TRUE(driver->ReadChassisState(&state));
  driver->Stop();
}

TEST(L1wDriverHw, VelocityCommandAxes) {
  SkipUnlessHwReady();
  if (!EnvTruthy("AUTODRIVER_L1W_HW_MOTION")) {
    GTEST_SKIP() << "set AUTODRIVER_L1W_HW_MOTION=1 for live velocity cmds";
  }

  auto driver = MakeHwDriver();
  ASSERT_TRUE(driver->Start());
  SleepMs(kHwActionGapMs);
  ASSERT_TRUE(driver->ApplyLocomotionIntent(
      autodriver::chassis::LocomotionIntent::kWheel));
  // Extra settle after standUp before the first move pulse.
  SleepMs(kHwActionGapMs);

  // Open-loop ~1 m per axis (time = distance / speed). Clear space around dog.
  // +x forward, -x back; +y left, -y right (body frame).
  // Velocity is re-sent every 200 ms so the SDK stays in move mode.
  EXPECT_TRUE(DriveAxisForMeters(*driver, kHwVxMps, 0.0, kHwTravelMeters))
      << "forward 1 m failed";
  EXPECT_TRUE(DriveAxisForMeters(*driver, -kHwVxMps, 0.0, kHwTravelMeters))
      << "backward 1 m failed";
  EXPECT_TRUE(DriveAxisForMeters(*driver, 0.0, kHwVyMps, kHwTravelMeters))
      << "left 1 m failed";
  EXPECT_TRUE(DriveAxisForMeters(*driver, 0.0, -kHwVyMps, kHwTravelMeters))
      << "right 1 m failed";

  autodriver::chassis::ChassisState state;
  EXPECT_TRUE(driver->ReadChassisState(&state));
  EXPECT_GE(state.battery_percent(), 0.0f);
  driver->Stop();
}

TEST(L1wDriverHw, WalkCrawlCancel) {
  SkipUnlessHwReady();
  if (!EnvTruthy("AUTODRIVER_L1W_HW_MOTION")) {
    GTEST_SKIP() << "set AUTODRIVER_L1W_HW_MOTION=1 to enable crawl";
  }

  auto driver = MakeHwDriver();
  ASSERT_TRUE(driver->Start());
  SleepMs(kHwActionGapMs);
  ASSERT_TRUE(driver->ApplyLocomotionIntent(
      autodriver::chassis::LocomotionIntent::kWalk));
  SleepMs(kHwActionGapMs);

  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.05);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));
  SleepMs(kHwActionGapMs);

  autodriver::chassis::ToolCommand cancel;
  cancel.name = "cancel_crawl";
  EXPECT_TRUE(driver->ApplyToolCommand(cancel));
  SleepMs(kHwActionGapMs);

  cmd.Clear();
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));
  SleepMs(kHwActionGapMs);
  driver->Stop();
}

TEST(L1wDriverHw, SpecialTools) {
  SkipUnlessHwReady();
  if (!EnvTruthy("AUTODRIVER_L1W_HW_SPECIAL")) {
    GTEST_SKIP() << "set AUTODRIVER_L1W_HW_SPECIAL=1 for shake/squat/climb "
                    "(clear space around the dog)";
  }

  auto driver = MakeHwDriver();
  ASSERT_TRUE(driver->Start());
  SleepMs(kHwActionGapMs);

  autodriver::chassis::ToolCommand attitude;
  attitude.name = "attitude";
  attitude.value = "0,0,0,0";
  EXPECT_TRUE(driver->ApplyToolCommand(attitude));
  SleepMs(kHwActionGapMs);

  autodriver::chassis::ToolCommand shake;
  shake.name = "shake_hand";
  EXPECT_TRUE(driver->ApplyToolCommand(shake));
  SleepMs(kHwActionGapMs);

  autodriver::chassis::ToolCommand squat;
  squat.name = "rear_squat";
  EXPECT_TRUE(driver->ApplyToolCommand(squat));
  SleepMs(kHwActionGapMs);

  autodriver::chassis::ToolCommand climb;
  climb.name = "climb";
  climb.value = "0.03,0,0";
  EXPECT_TRUE(driver->ApplyToolCommand(climb));
  SleepMs(kHwActionGapMs);

  autodriver::chassis::ToolCommand cancel_climb;
  cancel_climb.name = "cancel_climb";
  EXPECT_TRUE(driver->ApplyToolCommand(cancel_climb));
  SleepMs(kHwActionGapMs);

  driver->Stop();
}

}  // namespace

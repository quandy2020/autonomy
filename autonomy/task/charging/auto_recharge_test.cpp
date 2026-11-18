/*
 * Copyright 2026 The Openbot Authors
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
 * @file auto_recharge_test.cpp
 * @brief Hysteresis / interlock tests for AutoRechargePolicy.
 */

#include "autonomy/task/charging/auto_recharge.hpp"

#include <gtest/gtest.h>

namespace {

using autonomy::task::charging::AutoRechargeIntent;
using autonomy::task::charging::AutoRechargePolicy;
using autonomy::task::charging::AutoRechargeView;
using autonomy::task::charging::BatterySocPercent;
namespace proto = autonomy::task::proto;

AutoRechargePolicy MakePolicy() {
    proto::AutoRechargeOptions o;
    o.set_enable(true);
    o.set_return_percent(20.f);
    o.set_resume_percent(95.f);
    o.set_auto_undock(true);
    o.set_skip_if_teleop(true);
    o.set_max_consecutive_failures(3);
    o.set_cooldown_sec(60.f);
    return AutoRechargePolicy(o);
}

AutoRechargeView BaseView() {
    AutoRechargeView v;
    v.enable = true;
    v.battery_fresh = true;
    v.soc_percent = 50.f;
    v.now = std::chrono::steady_clock::now();
    return v;
}

TEST(BatterySocPercent, ScalesUnitInterval) {
    automsgs::msgs::sensor_msgs::BatteryState msg;
    msg.set_percentage(0.42f);
    EXPECT_NEAR(BatterySocPercent(msg), 42.f, 1e-3);
}

TEST(BatterySocPercent, AcceptsAlreadyPercent) {
    automsgs::msgs::sensor_msgs::BatteryState msg;
    msg.set_percentage(88.f);
    EXPECT_NEAR(BatterySocPercent(msg), 88.f, 1e-3);
}

TEST(AutoRechargePolicy, DocksAtOrBelowReturn) {
    const auto policy = MakePolicy();
    auto view = BaseView();
    view.soc_percent = 20.f;
    EXPECT_EQ(policy.Evaluate(view), AutoRechargeIntent::kDock);
    view.soc_percent = 19.f;
    EXPECT_EQ(policy.Evaluate(view), AutoRechargeIntent::kDock);
    view.soc_percent = 21.f;
    EXPECT_EQ(policy.Evaluate(view), AutoRechargeIntent::kIdle);
}

TEST(AutoRechargePolicy, SkipsTeleopAndStaleBattery) {
    const auto policy = MakePolicy();
    auto view = BaseView();
    view.soc_percent = 10.f;
    view.teleop_active = true;
    EXPECT_EQ(policy.Evaluate(view), AutoRechargeIntent::kIdle);
    view.teleop_active = false;
    view.battery_fresh = false;
    EXPECT_EQ(policy.Evaluate(view), AutoRechargeIntent::kIdle);
}

TEST(AutoRechargePolicy, CooldownAndFailureCap) {
    const auto policy = MakePolicy();
    auto view = BaseView();
    view.soc_percent = 10.f;
    view.cooldown_until = view.now + std::chrono::seconds(10);
    EXPECT_EQ(policy.Evaluate(view), AutoRechargeIntent::kIdle);
    view.cooldown_until = {};
    view.consecutive_failures = 3;
    EXPECT_EQ(policy.Evaluate(view), AutoRechargeIntent::kIdle);
    view.soc_percent = 96.f;
    EXPECT_EQ(policy.Evaluate(view), AutoRechargeIntent::kIdle);
}

TEST(AutoRechargePolicy, UndockWhenHoldingAtResume) {
    const auto policy = MakePolicy();
    auto view = BaseView();
    view.charging_active = true;
    view.holding_charge = true;
    view.soc_percent = 95.f;
    EXPECT_EQ(policy.Evaluate(view), AutoRechargeIntent::kUndock);
    view.soc_percent = 80.f;
    EXPECT_EQ(policy.Evaluate(view), AutoRechargeIntent::kIdle);
}

TEST(AutoRechargePolicy, DoesNotDoubleSubmitWhileDocking) {
    const auto policy = MakePolicy();
    auto view = BaseView();
    view.soc_percent = 10.f;
    view.charging_active = true;
    view.holding_charge = false;
    EXPECT_EQ(policy.Evaluate(view), AutoRechargeIntent::kIdle);
}

}  // namespace

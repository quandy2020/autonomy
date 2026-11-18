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
 * @file auto_recharge.cpp
 * @brief Production auto-return-to-dock policy and supervisor (implementation).
 */

#include "autonomy/task/charging/auto_recharge.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace task {
namespace charging {
namespace {

using Clock = std::chrono::steady_clock;

::autonomy::task::proto::AutoRechargeOptions WithDefaults(
    ::autonomy::task::proto::AutoRechargeOptions o) {
    if (o.return_percent() <= 0.f) {
        o.set_return_percent(20.f);
    }
    if (o.resume_percent() <= 0.f) {
        o.set_resume_percent(95.f);
    }
    if (o.battery_stale_sec() <= 0.f) {
        o.set_battery_stale_sec(5.f);
    }
    if (o.cooldown_sec() <= 0.f) {
        o.set_cooldown_sec(60.f);
    }
    if (o.max_consecutive_failures() <= 0) {
        o.set_max_consecutive_failures(3);
    }
    if (o.max_hold_charge_sec() <= 0.f) {
        o.set_max_hold_charge_sec(14400.f);
    }
    if (o.battery_state_topic().empty()) {
        o.set_battery_state_topic("/battery_state");
    }
    if (o.resume_percent() <= o.return_percent()) {
        o.set_resume_percent(std::min(100.f, o.return_percent() + 30.f));
    }
    return o;
}

}  // namespace

float BatterySocPercent(
    const ::automsgs::msgs::sensor_msgs::BatteryState& msg) {
    float percent = std::numeric_limits<float>::quiet_NaN();
    if (std::isfinite(msg.percentage()) && msg.percentage() >= 0.f) {
        percent = msg.percentage() > 1.5f ? msg.percentage()
                                          : msg.percentage() * 100.f;
    } else if (msg.capacity() > 0.f && std::isfinite(msg.charge())) {
        percent = 100.f * msg.charge() / msg.capacity();
    }
    if (!std::isfinite(percent)) {
        return percent;
    }
    return std::clamp(percent, 0.f, 100.f);
}

AutoRechargePolicy::AutoRechargePolicy(
    ::autonomy::task::proto::AutoRechargeOptions options)
    : options_(WithDefaults(std::move(options))) {
    return_percent_ = options_.return_percent();
    resume_percent_ = options_.resume_percent();
    max_failures_ = options_.max_consecutive_failures();
    auto_undock_ = options_.auto_undock();
    skip_teleop_ = options_.skip_if_teleop();
}

AutoRechargeIntent AutoRechargePolicy::Evaluate(
    const AutoRechargeView& view) const {
    if (!view.enable || !view.battery_fresh || !std::isfinite(view.soc_percent)) {
        return AutoRechargeIntent::kIdle;
    }
    if (skip_teleop_ && view.teleop_active) {
        return AutoRechargeIntent::kIdle;
    }
    if (view.now < view.cooldown_until) {
        return AutoRechargeIntent::kIdle;
    }
    if (view.consecutive_failures >= max_failures_ &&
        view.soc_percent < resume_percent_) {
        return AutoRechargeIntent::kIdle;
    }
    if (view.charging_active) {
        if (auto_undock_ && view.holding_charge &&
            view.soc_percent >= resume_percent_) {
            return AutoRechargeIntent::kUndock;
        }
        return AutoRechargeIntent::kIdle;
    }
    if (view.soc_percent <= return_percent_) {
        return AutoRechargeIntent::kDock;
    }
    return AutoRechargeIntent::kIdle;
}

bool AutoRechargeSupervisor::Configure(
    const std::shared_ptr<autolink::Node>& node,
    const ::autonomy::task::proto::AutoRechargeOptions& options) {
    Stop();
    options_ = WithDefaults(options);
    policy_ = AutoRechargePolicy(options_);
    node_ = node;
    if (!options_.enable()) {
        AINFO << "auto-recharge disabled";
        return true;
    }
    if (!node_) {
        AERROR << "auto-recharge enable but node is null";
        return false;
    }
    battery_reader_ = node_->CreateReader<
        ::automsgs::msgs::sensor_msgs::BatteryState>(
        options_.battery_state_topic(),
        [this](const std::shared_ptr<::automsgs::msgs::sensor_msgs::BatteryState>&
                   msg) {
            if (!msg) {
                return;
            }
            const float soc = BatterySocPercent(*msg);
            soc_.store(soc);
            std::lock_guard<std::mutex> lock(mutex_);
            battery_stamp_ = Clock::now();
        });
    AINFO << "auto-recharge enable return=" << options_.return_percent()
          << "% resume=" << options_.resume_percent() << "% dock="
          << options_.preferred_dock_id();
    return true;
}

void AutoRechargeSupervisor::Start(SubmitFn submit, TeleopFn teleop_active,
                                   ActiveFn charging_active, PhaseFn phase,
                                   DockIdFn select_dock) {
    if (!options_.enable()) {
        return;
    }
    Stop();
    submit_ = std::move(submit);
    teleop_active_ = std::move(teleop_active);
    charging_active_ = std::move(charging_active);
    phase_ = std::move(phase);
    select_dock_ = std::move(select_dock);
    running_.store(true);
    worker_ = std::thread([this] { Loop(); });
}

void AutoRechargeSupervisor::Stop() {
    running_.store(false);
    if (worker_.joinable()) {
        worker_.join();
    }
    battery_reader_.reset();
}

void AutoRechargeSupervisor::NotifyFailure() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++consecutive_failures_;
    cooldown_until_ =
        Clock::now() + std::chrono::duration_cast<Clock::duration>(
                           std::chrono::duration<double>(options_.cooldown_sec()));
    AWARN << "auto-recharge failure streak=" << consecutive_failures_;
}

void AutoRechargeSupervisor::NotifyCancel() {
    std::lock_guard<std::mutex> lock(mutex_);
    cooldown_until_ =
        Clock::now() + std::chrono::duration_cast<Clock::duration>(
                           std::chrono::duration<double>(options_.cooldown_sec()));
}

void AutoRechargeSupervisor::NotifySuccess() {
    std::lock_guard<std::mutex> lock(mutex_);
    consecutive_failures_ = 0;
}

AutoRechargeView AutoRechargeSupervisor::SampleLocked() const {
    AutoRechargeView view;
    view.enable = options_.enable();
    view.now = Clock::now();
    view.cooldown_until = cooldown_until_;
    view.consecutive_failures = consecutive_failures_;
    const double stale = options_.battery_stale_sec();
    view.battery_fresh =
        battery_stamp_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(view.now - battery_stamp_).count() <=
            stale;
    view.soc_percent = soc_.load();
    return view;
}

void AutoRechargeSupervisor::Loop() {
    while (running_.load()) {
        AutoRechargeView view;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            view = SampleLocked();
        }
        view.teleop_active = teleop_active_ ? teleop_active_() : false;
        view.charging_active = charging_active_ ? charging_active_() : false;
        const auto phase =
            phase_ ? phase_() : ::autonomy::task::proto::DOCK_PHASE_NONE;
        view.holding_charge =
            phase == ::autonomy::task::proto::DOCK_PHASE_HOLD_CHARGE ||
            phase == ::autonomy::task::proto::DOCK_PHASE_WAIT_FOR_CHARGE;
        const auto intent = policy_.Evaluate(view);
        const auto now = Clock::now();
        if (intent != AutoRechargeIntent::kIdle &&
            last_command_.time_since_epoch().count() != 0 &&
            now - last_command_ < std::chrono::seconds(5)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        if (intent == AutoRechargeIntent::kDock && submit_) {
            ::autonomy::task::proto::ChargingGoal goal;
            goal.set_command(::autonomy::task::proto::DOCK_CMD_START);
            goal.set_use_dock_id(true);
            const std::string id = select_dock_ ? select_dock_() : std::string();
            if (!id.empty()) {
                goal.set_dock_station_id(id);
            }
            goal.set_battery_target_percent(options_.resume_percent());
            AINFO << "auto-recharge dock soc=" << view.soc_percent
                  << "% dock_id=" << id;
            if (!submit_(goal)) {
                NotifyFailure();
            } else {
                last_command_ = now;
            }
        } else if (intent == AutoRechargeIntent::kUndock && submit_) {
            ::autonomy::task::proto::ChargingGoal goal;
            goal.set_command(::autonomy::task::proto::DOCK_CMD_UNDOCK);
            AINFO << "auto-recharge undock soc=" << view.soc_percent << "%";
            if (submit_(goal)) {
                last_command_ = now;
            }
        }
        if (view.battery_fresh && view.soc_percent >= options_.resume_percent()) {
            NotifySuccess();
        }
        for (int i = 0; i < 10 && running_.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}

}  // namespace charging
}  // namespace task
}  // namespace autonomy

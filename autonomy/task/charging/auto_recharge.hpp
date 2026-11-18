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
 * @file auto_recharge.hpp
 * @brief Production auto-return-to-dock policy and supervisor.
 */

#ifndef AUTONOMY_TASK_CHARGING_AUTO_RECHARGE_HPP_
#define AUTONOMY_TASK_CHARGING_AUTO_RECHARGE_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <automsgs/msgs/sensor_msgs/battery_state.pb.h>
#include <automsgs/task/charging.pb.h>
#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"

namespace autonomy {
namespace task {
namespace charging {

/**
 * @brief Normalize BatteryState.percentage (0-1 or 0-100) to [0, 100].
 * @param[in] msg Battery sample.
 * @return SOC percent, or NaN when unknown.
 */
float BatterySocPercent(
    const ::automsgs::msgs::sensor_msgs::BatteryState& msg);

/**
 * @brief Decision output of AutoRechargePolicy.
 */
enum class AutoRechargeIntent { kIdle, kDock, kUndock };

/**
 * @brief Inputs sampled each supervisor tick (no Autolink).
 */
struct AutoRechargeView {
    bool enable{false};
    bool teleop_active{false};
    bool charging_active{false};
    bool holding_charge{false};
    bool battery_fresh{false};
    float soc_percent{0.f};
    int consecutive_failures{0};
    std::chrono::steady_clock::time_point now{};
    std::chrono::steady_clock::time_point cooldown_until{};
};

/**
 * @class autonomy::task::charging::AutoRechargePolicy
 * @brief Hysteresis + cooldown + teleop interlock. Pure function of View.
 */
class AutoRechargePolicy {
public:
    explicit AutoRechargePolicy(
        ::autonomy::task::proto::AutoRechargeOptions options = {});

    /**
     * @brief Evaluate one tick.
     * @param[in] view Latest robot/task/battery snapshot.
     * @return Dock, undock, or idle.
     */
    AutoRechargeIntent Evaluate(const AutoRechargeView& view) const;

    const ::autonomy::task::proto::AutoRechargeOptions& options() const {
        return options_;
    }

private:
    ::autonomy::task::proto::AutoRechargeOptions options_;
    float return_percent_{20.f};
    float resume_percent_{95.f};
    int max_failures_{3};
    bool auto_undock_{false};
    bool skip_teleop_{true};
};

/**
 * @class autonomy::task::charging::AutoRechargeSupervisor
 * @brief 2 Hz battery watcher that submits ChargingGoal through TaskServer.
 */
class AutoRechargeSupervisor {
public:
    using SubmitFn =
        std::function<bool(const ::autonomy::task::proto::ChargingGoal&)>;
    using TeleopFn = std::function<bool()>;
    using ActiveFn = std::function<bool()>;
    using PhaseFn = std::function<::autonomy::task::proto::DockPhase()>;
    using DockIdFn = std::function<std::string()>;

    /**
     * @brief Bind options and Autolink battery reader.
     * @param[in] node Autolink node.
     * @param[in] options AutoRechargeOptions from charging.pb.txt.
     * @return true when enabled and the reader opened, or when disabled.
     */
    bool Configure(const std::shared_ptr<autolink::Node>& node,
                   const ::autonomy::task::proto::AutoRechargeOptions& options);

    /**
     * @brief Start the worker thread.
     * @param[in] submit TaskServer::Submit(ChargingGoal).
     * @param[in] teleop_active true while teleop holds the robot.
     * @param[in] charging_active true while ChargingTask is running.
     * @param[in] phase Current dock phase.
     * @param[in] select_dock Preferred or nearest dock id.
     */
    void Start(SubmitFn submit, TeleopFn teleop_active, ActiveFn charging_active,
               PhaseFn phase, DockIdFn select_dock);

    /**
     * @brief Stop the worker and drop the battery reader.
     */
    void Stop();

    /**
     * @brief Record a failed dock attempt (extends cooldown).
     */
    void NotifyFailure();

    /**
     * @brief Record operator cancel (extends cooldown, avoids immediate re-dock).
     */
    void NotifyCancel();

    /**
     * @brief Record a successful dock or undock (clears failure streak).
     */
    void NotifySuccess();

    bool enabled() const { return options_.enable(); }
    float soc_percent() const { return soc_.load(); }

private:
    void Loop();
    AutoRechargeView SampleLocked() const;

    AutoRechargePolicy policy_;
    ::autonomy::task::proto::AutoRechargeOptions options_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<
        autolink::Reader<::automsgs::msgs::sensor_msgs::BatteryState>>
        battery_reader_;

    SubmitFn submit_;
    TeleopFn teleop_active_;
    ActiveFn charging_active_;
    PhaseFn phase_;
    DockIdFn select_dock_;

    mutable std::mutex mutex_;
    std::chrono::steady_clock::time_point battery_stamp_{};
    std::chrono::steady_clock::time_point cooldown_until_{};
    std::chrono::steady_clock::time_point last_command_{};
    int consecutive_failures_{0};
    std::atomic<float> soc_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<bool> running_{false};
    std::thread worker_;
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_CHARGING_AUTO_RECHARGE_HPP_

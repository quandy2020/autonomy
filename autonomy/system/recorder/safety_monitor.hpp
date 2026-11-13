/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file safety_monitor.hpp
 * @brief L4 independent process: observe /cmd_vel, latch-only, never publish velocity.
 *
 * Unsafe command → SafetyLatch (monitor MRM zeros cmd). Hash-chain every latch.
 */

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "autolink/node/node.hpp"
#include "automsgs/msgs/geometry_msgs/twist.pb.h"
#include "autonomy/system/recorder/hash_chain.hpp"
#include "autonomy/system/recorder/proto/recorder_options.pb.h"
#include "autonomy/system/recorder/rolling_buffer.hpp"
#include "autonomy/system/recorder/signals.hpp"
#include "autonomy/system/recorder/subscriber.hpp"
#include "autonomy/system/safety/safety_latch.hpp"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Outcome of EvaluateCommand. Never an in-path intercept of /cmd_vel.
 */
enum class SafetyVerdict {
    kPass = 0,      ///< Command within bounds.
    kFallback = 1,  ///< Latch / MRM fallback.
    kHandover = 2,  ///< Log handover (teleop already active).
};

/**
 * @brief Inputs to the latch-only command check.
 */
struct SafetyCheckInput {
    automsgs::msgs::geometry_msgs::Twist cmd;  ///< Observed /cmd_vel.
    bool latch_already{false};                 ///< Latch already set.
    bool teleop_active{false};                 ///< Teleop currently driving.
    double max_vx{2.0};                        ///< Linear speed limit, m/s.
    double max_wz{2.0};                        ///< Angular speed limit, rad/s.
};

/**
 * @brief Independent of planning/control: pass, latch-fallback, or log handover.
 * @param [in] in: Observed command and latch/teleop state.
 * @return SafetyVerdict.
 */
SafetyVerdict EvaluateCommand(const SafetyCheckInput& in);

/**
 * @brief L4 process between policy /cmd_vel and actuators.
 *
 * Never publishes velocity. Unsafe command → SafetyLatch (MRM zeros cmd).
 */
class SafetyMonitor {
public:
    /**
     * @brief Apply options and restore the hash chain if present.
     * @param [in] options: RecorderOptions (limits, chain path).
     * @return True if configuration is usable.
     */
    bool Configure(const proto::RecorderOptions& options);

    /**
     * @brief Attach bypass readers for /cmd_vel and teleop.
     * @param [in] node: Autolink node.
     * @return True if at least one reader attached.
     */
    bool Start(const std::shared_ptr<autolink::Node>& node);

    /**
     * @brief Persist a dirty hash chain if needed.
     */
    void SpinOnce();

    /**
     * @brief Detach readers and flush the chain.
     */
    void Shutdown();

    /**
     * @brief Last EvaluateCommand verdict.
     * @return SafetyVerdict.
     */
    SafetyVerdict last_verdict() const;

    /**
     * @brief Number of events in the in-memory hash chain.
     * @return Count.
     */
    size_t chain_size() const;

private:
    /**
     * @brief Parse a cmd_vel / teleop sample and maybe latch.
     * @param [in] sample: Bypass sample.
     */
    void OnSample(const Sample& sample);

    /**
     * @brief Apply verdict under mu_: latch file + hash-chain append.
     * @param [in] verdict: New verdict.
     * @param [in] detail: Reason text.
     * @param [in] clock: Dual clock.
     */
    void ApplyLocked(SafetyVerdict verdict, const std::string& detail,
                     const proto::DualClock& clock);

    proto::RecorderOptions options_;
    BypassSubscriber subscriber_;
    safety::SafetyLatch latch_;
    SafetyHashChain chain_;
    mutable std::mutex mu_;
    SafetyVerdict last_{SafetyVerdict::kPass};
    bool dirty_{false};        ///< Chain needs Save.
    std::string chain_path_;   ///< Persistence path.
    uint64_t seq_{0};          ///< SafetyEvent sequence.
};

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

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

/**
 * @file safety_monitor.cpp
 * @brief L4 latch-only /cmd_vel observer; never publishes velocity.
 */

#include "autonomy/system/recorder/safety_monitor.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <sstream>

#include <glog/logging.h>

#include "autonomy/system/recorder/clock.hpp"
#include "autonomy/system/recorder/version_tuple.hpp"

namespace autonomy {
namespace system {
namespace recorder {
namespace fs = std::filesystem;

SafetyVerdict EvaluateCommand(const SafetyCheckInput& in) {
    if (in.latch_already) {
        return SafetyVerdict::kPass;
    }
    if (in.teleop_active) {
        return SafetyVerdict::kHandover;
    }
    const double max_vx = in.max_vx > 0.0 ? in.max_vx : 2.0;
    const double max_wz = in.max_wz > 0.0 ? in.max_wz : 2.0;
    if (std::abs(in.cmd.linear().x()) > max_vx ||
        std::abs(in.cmd.angular().z()) > max_wz) {
        return SafetyVerdict::kFallback;
    }
    return SafetyVerdict::kPass;
}

bool SafetyMonitor::Configure(const proto::RecorderOptions& options) {
    options_ = options;
    if (options_.max_cmd_vx() <= 0.0) {
        options_.set_max_cmd_vx(2.0);
    }
    if (options_.max_cmd_wz() <= 0.0) {
        options_.set_max_cmd_wz(2.0);
    }
    if (options_.cmd_vel_channel().empty()) {
        options_.set_cmd_vel_channel("/cmd_vel");
    }
    if (options_.teleop_goal_channel().empty()) {
        options_.set_teleop_goal_channel("/autonomy/task/teleop/goal");
    }
    chain_path_ = options_.safety_chain_path();
    if (chain_path_.empty()) {
        std::string root = options_.output_dir();
        if (root.empty()) {
            if (const char* env = std::getenv("AUTONOMY_RECORD_ROOT");
                env != nullptr && env[0] != '\0') {
                root = env;
            } else if (const char* home = std::getenv("HOME");
                       home != nullptr && home[0] != '\0') {
                root = std::string(home) + "/.autonomy/records";
            } else {
                root = "/tmp/autonomy/records";
            }
        }
        chain_path_ = root + "/safety/chain.bin";
    }
    fs::create_directories(fs::path(chain_path_).parent_path());
    chain_ = SafetyHashChain(options_.robot_id());
    chain_.Load(chain_path_);
    return true;
}

bool SafetyMonitor::Start(const std::shared_ptr<autolink::Node>& node) {
    proto::RecorderOptions sub = options_;
    sub.clear_channels();
    sub.add_channels()->set_name(options_.cmd_vel_channel());
    if (!options_.teleop_goal_channel().empty()) {
        sub.add_channels()->set_name(options_.teleop_goal_channel());
    }
    return subscriber_.Attach(
        node, sub, [this](Sample sample) { OnSample(std::move(sample)); });
}

void SafetyMonitor::SpinOnce() {
    std::lock_guard<std::mutex> lock(mu_);
    if (!dirty_) {
        return;
    }
    if (chain_.Save(chain_path_)) {
        dirty_ = false;
    }
}

void SafetyMonitor::Shutdown() {
    subscriber_.Reset();
    std::lock_guard<std::mutex> lock(mu_);
    if (dirty_) {
        chain_.Save(chain_path_);
        dirty_ = false;
    }
}

SafetyVerdict SafetyMonitor::last_verdict() const {
    std::lock_guard<std::mutex> lock(mu_);
    return last_;
}

size_t SafetyMonitor::chain_size() const {
    std::lock_guard<std::mutex> lock(mu_);
    return chain_.size();
}

void SafetyMonitor::OnSample(const Sample& sample) {
    std::lock_guard<std::mutex> lock(mu_);
    if (sample.channel == options_.teleop_goal_channel()) {
        ApplyLocked(SafetyVerdict::kHandover, "teleop", sample.clock);
        return;
    }
    const auto twist = ParseTwist(sample);
    if (!twist) {
        return;
    }
    SafetyCheckInput in;
    in.cmd = *twist;
    in.latch_already = latch_.IsLatched();
    in.teleop_active = false;
    in.max_vx = options_.max_cmd_vx();
    in.max_wz = options_.max_cmd_wz();
    ApplyLocked(EvaluateCommand(in), sample.channel, sample.clock);
}

void SafetyMonitor::ApplyLocked(SafetyVerdict verdict, const std::string& detail,
                                const proto::DualClock& clock) {
    last_ = verdict;
    if (verdict == SafetyVerdict::kPass) {
        return;
    }
    proto::SafetyEvent event;
    std::ostringstream id;
    id << "sm-" << ++seq_ << "-" << clock.monotonic_ns();
    event.set_event_id(id.str());
    event.set_kind(verdict == SafetyVerdict::kFallback ? "cmd_limit"
                                                       : "handover");
    event.set_bt_node(detail);
    *event.mutable_clock() = clock.monotonic_ns() != 0 ? clock : NowDualClock();
    *event.mutable_version() = MakeVersionTuple(options_).tuple;
    chain_.Append(event);
    dirty_ = true;
    if (verdict == SafetyVerdict::kFallback) {
        latch_.SetLatched(true, "safety_monitor_cmd_limit");
        LOG(WARNING) << "safety_monitor fallback latch cmd_limit " << detail;
    } else {
        LOG(INFO) << "safety_monitor handover " << detail;
    }
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

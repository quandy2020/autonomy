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
 * @file trigger.cpp
 * @brief L1 trigger engine: BT, latch, deviation, timeout, task, loc quality.
 */

#include "autonomy/system/recorder/trigger.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

namespace autonomy {
namespace system {
namespace recorder {
namespace {

bool ContainsIgnoreCase(const std::string& haystack, const std::string& needle) {
    if (needle.empty() || haystack.size() < needle.size()) {
        return false;
    }
    auto it = std::search(haystack.begin(), haystack.end(), needle.begin(),
                          needle.end(), [](char a, char b) {
                              return std::tolower(static_cast<unsigned char>(a)) ==
                                     std::tolower(static_cast<unsigned char>(b));
                          });
    return it != haystack.end();
}

bool IsRecoverNode(const std::string& name) {
    return ContainsIgnoreCase(name, "recover");
}

double Mean(const std::vector<double>& v) {
    double s = 0;
    for (double x : v) {
        s += x;
    }
    return v.empty() ? 0.0 : s / static_cast<double>(v.size());
}

double Stddev(const std::vector<double>& v, double mean) {
    if (v.size() < 2) {
        return 0.0;
    }
    double s = 0;
    for (double x : v) {
        const double d = x - mean;
        s += d * d;
    }
    return std::sqrt(s / static_cast<double>(v.size() - 1));
}

}  // namespace

TriggerEngine::TriggerEngine(const proto::RecorderOptions& options)
    : options_(options) {
    for (const auto& ch : options_.channels()) {
        if (ch.timeout_sec() > 0.0) {
            timeouts_sec_[ch.name()] = ch.timeout_sec();
        }
    }
}

proto::TriggerEvent TriggerEngine::Make(proto::TriggerKind kind,
                                        const proto::DualClock& clock,
                                        const std::string& detail,
                                        const std::string& channel) const {
    proto::TriggerEvent event;
    event.set_kind(kind);
    *event.mutable_clock() = clock;
    event.set_detail(detail);
    event.set_channel(channel);
    std::ostringstream id;
    id << "trg_" << static_cast<int>(kind) << "_" << clock.monotonic_ns();
    event.set_evidence_node_id(id.str());
    return event;
}

std::optional<proto::TriggerEvent> TriggerEngine::OnSample(const Sample& sample) {
    last_seen_ns_[sample.channel] = sample.clock.monotonic_ns();
    return std::nullopt;
}

std::optional<proto::TriggerEvent> TriggerEngine::OnBtStatus(
    const std::string& node_name, const std::string& /*previous*/,
    const std::string& current, const proto::DualClock& clock) {
    if (IsRecoverNode(node_name) && current == "RUNNING") {
        return Make(proto::TRIGGER_BT_RECOVER, clock,
                    "BT entered recover: " + node_name, node_name);
    }
    if (current == "FAILURE" &&
        (ContainsIgnoreCase(node_name, "Navigate") ||
         ContainsIgnoreCase(node_name, "Localization") ||
         ContainsIgnoreCase(node_name, "Mapping") ||
         ContainsIgnoreCase(node_name, "Charging") ||
         ContainsIgnoreCase(node_name, "Dock") ||
         ContainsIgnoreCase(node_name, "Tracking") ||
         ContainsIgnoreCase(node_name, "Follow") ||
         ContainsIgnoreCase(node_name, "Exploration") ||
         ContainsIgnoreCase(node_name, "Voice"))) {
        return Make(proto::TRIGGER_TASK_FAILURE, clock,
                    "task node failed: " + node_name, node_name);
    }
    return std::nullopt;
}

std::optional<proto::TriggerEvent> TriggerEngine::OnSafetyLatch(
    bool latched, const std::string& reason, const proto::DualClock& clock) {
    if (latched && !estop_latched_) {
        estop_latched_ = true;
        return Make(proto::TRIGGER_ESTOP, clock,
                    reason.empty() ? "emergency_stop" : reason, "safety_latch");
    }
    if (!latched) {
        estop_latched_ = false;
    }
    return std::nullopt;
}

std::optional<proto::TriggerEvent> TriggerEngine::OnPathDeviation(
    double meters, const proto::DualClock& clock) {
    const double limit =
        options_.path_deviate_m() > 0.0 ? options_.path_deviate_m() : 0.5;
    if (meters >= limit) {
        std::ostringstream d;
        d << "path deviation " << meters << " m >= " << limit;
        return Make(proto::TRIGGER_PATH_DEVIATE, clock, d.str(), "/plan");
    }
    return std::nullopt;
}

std::optional<proto::TriggerEvent> TriggerEngine::OnCost(
    double value, const proto::DualClock& clock) {
    costs_.push_back(value);
    if (costs_.size() > 256) {
        costs_.erase(costs_.begin(), costs_.begin() + 64);
    }
    if (costs_.size() < 8) {
        return std::nullopt;
    }
    const double mean = Mean(costs_);
    const double sd = Stddev(costs_, mean);
    const double sigma =
        options_.cost_spike_sigma() > 0.0 ? options_.cost_spike_sigma() : 3.0;
    if (sd > 0.0 && (value - mean) >= sigma * sd) {
        std::ostringstream d;
        d << "controller cost " << value << " mean " << mean << " sd " << sd;
        return Make(proto::TRIGGER_COST_SPIKE, clock, d.str(), "/control/cost");
    }
    return std::nullopt;
}

std::optional<proto::TriggerEvent> TriggerEngine::OnTakeover(
    const proto::DualClock& clock, const std::string& channel) {
    return Make(proto::TRIGGER_TAKEOVER, clock, "human takeover", channel);
}

std::optional<proto::TriggerEvent> TriggerEngine::OnTaskFailure(
    const proto::DualClock& clock, const std::string& detail,
    const std::string& channel) {
    return Make(proto::TRIGGER_TASK_FAILURE, clock, detail, channel);
}

std::optional<proto::TriggerEvent> TriggerEngine::OnLocalizationQuality(
    double quality, const proto::DualClock& clock) {
    const double floor = options_.localization_quality_floor() > 0.0
                             ? options_.localization_quality_floor()
                             : 0.3;
    if (quality > 0.0 && quality < floor && !loc_quality_latched_) {
        loc_quality_latched_ = true;
        std::ostringstream d;
        d << "localization_quality " << quality << " < " << floor;
        return Make(proto::TRIGGER_TASK_FAILURE, clock, d.str(),
                    "/autonomy/task/localization/feedback");
    }
    if (quality >= floor) {
        loc_quality_latched_ = false;
    }
    return std::nullopt;
}

std::vector<proto::TriggerEvent> TriggerEngine::PollTimeouts(
    const proto::DualClock& now) {
    std::vector<proto::TriggerEvent> out;
    for (const auto& [channel, timeout] : timeouts_sec_) {
        auto it = last_seen_ns_.find(channel);
        if (it == last_seen_ns_.end()) {
            continue;
        }
        const double age =
            static_cast<double>(now.monotonic_ns() - it->second) / 1e9;
        if (age > timeout) {
            std::ostringstream d;
            d << channel << " timeout age=" << age << "s";
            out.push_back(Make(proto::TRIGGER_CHANNEL_TIMEOUT, now, d.str(),
                               channel));
            it->second = now.monotonic_ns();
        }
    }
    return out;
}

proto::Priority TriggerEngine::Classify(
    const std::vector<proto::TriggerEvent>& events, bool task_success) const {
    for (const auto& e : events) {
        if (e.kind() == proto::TRIGGER_ESTOP ||
            e.kind() == proto::TRIGGER_BT_RECOVER ||
            e.kind() == proto::TRIGGER_TASK_FAILURE) {
            return proto::PRIORITY_P0;
        }
        if (e.kind() == proto::TRIGGER_TAKEOVER) {
            return proto::PRIORITY_P1;
        }
        if (e.kind() == proto::TRIGGER_COST_SPIKE ||
            e.kind() == proto::TRIGGER_PATH_DEVIATE ||
            e.kind() == proto::TRIGGER_CHANNEL_TIMEOUT) {
            return proto::PRIORITY_P2;
        }
    }
    return task_success ? proto::PRIORITY_P3 : proto::PRIORITY_P0;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

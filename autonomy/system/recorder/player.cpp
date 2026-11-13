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
 * @file player.cpp
 * @brief SIL/HIL MCAP player and recorded-vs-replayed action compare.
 */

#include "autonomy/system/recorder/player.hpp"

#include <chrono>
#include <string>
#include <thread>
#include <unordered_map>

#include "autolink/common/log.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/node/writer.hpp"
#include "autonomy/system/recorder/mcap_writer.hpp"
#include "autonomy/system/recorder/replay.hpp"
#include "autonomy/system/recorder/signals.hpp"

namespace autonomy {
namespace system {
namespace recorder {

std::vector<proto::ActionEvent> ActionsFromMcap(const std::string& mcap_path) {
    std::vector<proto::ActionEvent> out;
    uint32_t seq = 0;
    for (const auto& m : RecoverMcap(mcap_path)) {
        if (m.topic.find("cmd_vel") == std::string::npos) {
            continue;
        }
        Sample s;
        s.channel = m.topic;
        s.payload = m.payload;
        if (auto tw = ParseTwist(s)) {
            out.push_back(TwistToAction(*tw, seq++,
                                        static_cast<int64_t>(m.log_time_ns)));
        }
    }
    return out;
}

proto::ReplayReport CompareMcaps(const std::string& recorded_mcap,
                                 const std::string& replayed_mcap,
                                 double match_threshold, bool factual) {
    const auto rec = ActionsFromMcap(recorded_mcap);
    const auto rep = ActionsFromMcap(replayed_mcap);
    if (factual) {
        return FactualReplay(recorded_mcap, rec.size() == rep.size(), rec, rep,
                             match_threshold);
    }
    return InterventionalReplay(recorded_mcap, true, rec, rep, match_threshold);
}

ReplayProfile ParseReplayProfile(const std::string& name) {
    if (name == "sensor_in") {
        return ReplayProfile::kSensorIn;
    }
    if (name == "mid_stack") {
        return ReplayProfile::kMidStack;
    }
    return ReplayProfile::kOpenLoop;
}

bool SkipReplayTopic(ReplayProfile profile, const std::string& topic) {
    if (profile == ReplayProfile::kOpenLoop) {
        return false;
    }
    if (topic.find("cmd_vel") != std::string::npos) {
        return true;
    }
    if (profile == ReplayProfile::kMidStack) {
        return false;
    }
    return topic == "/plan" ||
           topic.find("behavior_tree") != std::string::npos ||
           topic.find("/autonomy/task/") != std::string::npos ||
           topic.find("/perception/") != std::string::npos;
}

bool PlayMcap(const std::shared_ptr<autolink::Node>& node,
              const std::string& mcap_path, double rate,
              ReplayProfile profile) {
    if (!node) {
        return false;
    }
    const auto msgs = RecoverMcap(mcap_path);
    if (msgs.empty()) {
        AWARN << "recorder replay: empty or unreadable " << mcap_path;
        return false;
    }
    std::unordered_map<
        std::string, std::shared_ptr<autolink::Writer<autolink::message::RawMessage>>>
        writers;
    for (const auto& m : msgs) {
        if (m.topic.empty() || writers.count(m.topic)) {
            continue;
        }
        if (SkipReplayTopic(profile, m.topic)) {
            continue;
        }
        writers[m.topic] =
            node->CreateWriter<autolink::message::RawMessage>(m.topic);
    }
    int64_t prev = 0;
    for (const auto& m : msgs) {
        if (SkipReplayTopic(profile, m.topic)) {
            continue;
        }
        auto it = writers.find(m.topic);
        if (it == writers.end() || !it->second) {
            continue;
        }
        if (rate > 0.0 && prev > 0 &&
            static_cast<int64_t>(m.log_time_ns) > prev) {
            const auto dt_ns = static_cast<int64_t>(
                static_cast<double>(m.log_time_ns - static_cast<uint64_t>(prev)) /
                rate);
            if (dt_ns > 0) {
                std::this_thread::sleep_for(std::chrono::nanoseconds(dt_ns));
            }
        }
        prev = static_cast<int64_t>(m.log_time_ns);
        auto raw = std::make_shared<autolink::message::RawMessage>(
            m.payload, m.log_time_ns);
        it->second->Write(raw);
    }
    AINFO << "recorder replay: published " << msgs.size() << " messages from "
          << mcap_path;
    return true;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

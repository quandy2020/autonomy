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
 * @file episode.cpp
 * @brief Episode open/close from BT root and goals; session-mode cap.
 */

#include "autonomy/system/recorder/episode.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace autonomy {
namespace system {
namespace recorder {
namespace {

bool ContainsIgnoreCase(const std::string& haystack, const std::string& needle) {
    auto it = std::search(haystack.begin(), haystack.end(), needle.begin(),
                          needle.end(), [](char a, char b) {
                              return std::tolower(static_cast<unsigned char>(a)) ==
                                     std::tolower(static_cast<unsigned char>(b));
                          });
    return it != haystack.end();
}

bool IsTaskRoot(const std::string& name) {
    return ContainsIgnoreCase(name, "NavigateTo") ||
           ContainsIgnoreCase(name, "NavigateThrough") ||
           ContainsIgnoreCase(name, "Localization") ||
           ContainsIgnoreCase(name, "Mapping") ||
           ContainsIgnoreCase(name, "Charging") ||
           ContainsIgnoreCase(name, "Dock") ||
           ContainsIgnoreCase(name, "Tracking") ||
           ContainsIgnoreCase(name, "Follow") ||
           ContainsIgnoreCase(name, "Exploration") ||
           ContainsIgnoreCase(name, "Voice");
}

bool IsRecoverNode(const std::string& name) {
    return ContainsIgnoreCase(name, "recover");
}

bool IsCloseStatus(const std::string& status) {
    return status == "SUCCESS" || status == "FAILURE";
}

}  // namespace

std::string EpisodeTracker::NextId(const proto::DualClock& clock) {
    const time_t sec = static_cast<time_t>(clock.wall_utc_ns() / 1000000000LL);
    std::tm tm{};
    gmtime_r(&sec, &tm);
    std::ostringstream oss;
    oss << "ep_" << std::put_time(&tm, "%Y%m%dT%H%M%SZ") << "_" << ++counter_;
    return oss.str();
}

void EpisodeTracker::Open(const proto::DualClock& clock, const std::string& root) {
    state_ = EpisodeState{};
    state_.open = true;
    state_.episode_id = NextId(clock);
    state_.bt_root = root;
    state_.start = clock;
}

void EpisodeTracker::OnGoal(const proto::DualClock& clock) {
    if (!state_.open) {
        Open(clock, "goal");
    }
}

std::optional<EpisodeState> EpisodeTracker::OnBtStatus(
    const BtTransition& transition) {
    if (state_.open) {
        state_.transitions.push_back(transition);
        constexpr size_t kMaxTransitions = 2048;
        if (session_mode_ && state_.transitions.size() > kMaxTransitions) {
            state_.transitions.erase(
                state_.transitions.begin(),
                state_.transitions.begin() +
                    static_cast<std::ptrdiff_t>(kMaxTransitions / 2));
        }
    }

    if (!state_.open && IsTaskRoot(transition.node_name) &&
        transition.previous == "IDLE" && transition.current == "RUNNING") {
        Open(transition.clock, transition.node_name);
        state_.transitions.push_back(transition);
        return std::nullopt;
    }

    if (!state_.open) {
        return std::nullopt;
    }

    if (session_mode_) {
        return std::nullopt;
    }

    if (IsRecoverNode(transition.node_name) &&
        transition.current == "RUNNING") {
        return Close(transition.clock, "recover");
    }
    if (IsTaskRoot(transition.node_name) &&
        IsCloseStatus(transition.current)) {
        return Close(transition.clock, transition.current);
    }
    if (transition.node_name == state_.bt_root &&
        IsCloseStatus(transition.current)) {
        return Close(transition.clock, transition.current);
    }
    return std::nullopt;
}

std::optional<EpisodeState> EpisodeTracker::Close(const proto::DualClock& clock,
                                                  const std::string& /*reason*/) {
    if (!state_.open) {
        return std::nullopt;
    }
    EpisodeState closed = state_;
    closed.open = false;
    state_ = EpisodeState{};
    (void)clock;
    return closed;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

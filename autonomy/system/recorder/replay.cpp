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
 * @file replay.cpp
 * @brief L6 factual / interventional / pending replay reports.
 */

#include "autonomy/system/recorder/replay.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace autonomy {
namespace system {
namespace recorder {
namespace {

bool ParamsClose(const std::string& a, const std::string& b, double tol) {
    if (a == b) {
        return true;
    }
    char* ea = nullptr;
    char* eb = nullptr;
    const double da = std::strtod(a.c_str(), &ea);
    const double db = std::strtod(b.c_str(), &eb);
    if (ea != a.c_str() && eb != b.c_str()) {
        return std::fabs(da - db) <= tol;
    }
    return false;
}

}  // namespace

double ActionMatch(const std::vector<proto::ActionEvent>& recorded,
                   const std::vector<proto::ActionEvent>& replayed,
                   double param_tolerance) {
    if (recorded.empty()) {
        return replayed.empty() ? 1.0 : 0.0;
    }
    size_t j = 0;
    size_t matched = 0;
    for (const auto& rec : recorded) {
        bool found = false;
        while (j < replayed.size()) {
            const auto& rep = replayed[j];
            ++j;
            if (rep.type() == rec.type() && rep.target() == rec.target() &&
                ParamsClose(rep.params(), rec.params(), param_tolerance)) {
                found = true;
                break;
            }
        }
        if (found) {
            ++matched;
        }
    }
    return static_cast<double>(matched) / static_cast<double>(recorded.size());
}

proto::ReplayReport FactualReplay(const std::string& episode_id, bool bit_exact,
                                  const std::vector<proto::ActionEvent>& recorded,
                                  const std::vector<proto::ActionEvent>& replayed,
                                  double match_threshold) {
    proto::ReplayReport report;
    report.set_episode_id(episode_id);
    report.set_mode(proto::REPLAY_FACTUAL);
    report.set_bit_exact(bit_exact);
    report.set_action_match(ActionMatch(recorded, replayed, 1e-6));
    report.set_valid(bit_exact && report.action_match() >= match_threshold);
    if (!report.valid()) {
        report.set_reason(bit_exact ? "action_match" : "not_bit_exact");
    }
    return report;
}

proto::ReplayReport PendingReplay(const std::string& episode_id) {
    proto::ReplayReport report;
    report.set_episode_id(episode_id);
    report.set_mode(proto::REPLAY_UNSPECIFIED);
    report.set_valid(false);
    report.set_reason("pending_replay");
    return report;
}

proto::ReplayReport InterventionalReplay(
    const std::string& episode_id, bool improved,
    const std::vector<proto::ActionEvent>& recorded,
    const std::vector<proto::ActionEvent>& replayed, double match_threshold) {
    proto::ReplayReport report;
    report.set_episode_id(episode_id);
    report.set_mode(proto::REPLAY_INTERVENTIONAL);
    report.set_intervention_improved(improved);
    report.set_action_match(ActionMatch(recorded, replayed, 1e-3));
    report.set_valid(report.action_match() >= match_threshold);
    if (!report.valid()) {
        report.set_reason("action_match");
    }
    return report;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

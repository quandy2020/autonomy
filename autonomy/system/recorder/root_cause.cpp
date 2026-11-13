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
 * @file root_cause.cpp
 * @brief Rank predecessors of the last FAILURE; optional hypothesis reroll.
 */

#include "autonomy/system/recorder/root_cause.hpp"

#include <algorithm>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace autonomy {
namespace system {
namespace recorder {
namespace {

std::vector<std::string> Predecessors(const proto::EventGraph& graph,
                                      const std::string& id) {
    std::unordered_map<std::string, std::vector<std::string>> rev;
    for (const auto& e : graph.edges()) {
        rev[e.to_id()].push_back(e.from_id());
    }
    std::vector<std::string> out;
    std::unordered_set<std::string> seen;
    std::vector<std::string> stack{id};
    while (!stack.empty()) {
        const std::string cur = stack.back();
        stack.pop_back();
        for (const auto& p : rev[cur]) {
            if (seen.insert(p).second) {
                out.push_back(p);
                stack.push_back(p);
            }
        }
    }
    return out;
}

}  // namespace

std::vector<proto::RootCauseCard> RankRootCauses(const proto::EventGraph& graph) {
    std::vector<proto::RootCauseCard> cards;
    if (graph.failure_source_id().empty()) {
        return cards;
    }
    std::unordered_map<std::string, const proto::EvidenceNode*> by_id;
    for (const auto& n : graph.nodes()) {
        by_id[n.id()] = &n;
    }
    const auto* fail = by_id[graph.failure_source_id()];
    if (fail == nullptr) {
        return cards;
    }
    auto preds = Predecessors(graph, fail->id());
    proto::RootCauseCard primary;
    primary.set_hypothesis_id("h0");
    primary.set_evidence_node_id(fail->id());
    primary.set_statement("Failure source: " + fail->label());
    primary.set_confidence(1.0);
    cards.push_back(primary);

    const size_t n = preds.size();
    for (size_t i = 0; i < preds.size(); ++i) {
        const auto* node = by_id[preds[i]];
        if (node == nullptr) {
            continue;
        }
        proto::RootCauseCard card;
        std::ostringstream hid;
        hid << "h" << (i + 1);
        card.set_hypothesis_id(hid.str());
        card.set_evidence_node_id(node->id());
        card.set_statement("Upstream of failure: " + node->label());
        card.set_confidence(1.0 / static_cast<double>(i + 2));
        cards.push_back(card);
        (void)n;
    }
    std::sort(cards.begin(), cards.end(),
              [](const proto::RootCauseCard& a, const proto::RootCauseCard& b) {
                  return a.confidence() > b.confidence();
              });
    return cards;
}

bool RerollHypothesis(const CandidateFn& candidate,
                      const std::vector<HistoricalEpisode>& fail_set,
                      const std::vector<HistoricalEpisode>& success_set,
                      double regression_limit, proto::RootCauseCard* card) {
    if (card == nullptr || !candidate) {
        return false;
    }
    int fail_improved = 0;
    for (const auto& ep : fail_set) {
        if (candidate(ep)) {
            ++fail_improved;
        }
    }
    int success_regressed = 0;
    for (const auto& ep : success_set) {
        if (!candidate(ep)) {
            ++success_regressed;
        }
    }
    const double df = fail_set.empty()
                          ? 0.0
                          : static_cast<double>(fail_improved) /
                                static_cast<double>(fail_set.size());
    const double ds = success_set.empty()
                          ? 0.0
                          : static_cast<double>(success_regressed) /
                                static_cast<double>(success_set.size());
    card->set_delta_fail_set(df);
    card->set_delta_success_set(-ds);
    const bool passed = df > 0.0 && ds <= regression_limit;
    card->set_reroll_passed(passed);
    return passed;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

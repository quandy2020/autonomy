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
 * @file event_graph.cpp
 * @brief L2 graph: temporal + 2 s context edges over BT and triggers.
 */

#include "autonomy/system/recorder/event_graph.hpp"

#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <vector>

namespace autonomy {
namespace system {
namespace recorder {
namespace {

constexpr int64_t kContextWindowNs = 2'000'000'000;

std::string NodeId(const std::string& prefix, size_t i,
                   int64_t monotonic_ns) {
    std::ostringstream oss;
    oss << prefix << "_" << i << "_" << monotonic_ns;
    return oss.str();
}

proto::NodeKind KindFromBt(const std::string& current) {
    if (current == "FAILURE") {
        return proto::NODE_FAILURE_SOURCE;
    }
    if (current == "SUCCESS") {
        return proto::NODE_OUTCOME;
    }
    if (current == "RUNNING") {
        return proto::NODE_DECISION;
    }
    return proto::NODE_OBSERVATION;
}

struct BtRef {
    std::string id;
    int64_t ns{0};
};

}  // namespace

proto::EventGraph BuildEventGraph(
    const std::vector<BtTransition>& transitions,
    const std::vector<proto::TriggerEvent>& triggers) {
    proto::EventGraph graph;
    std::vector<BtRef> bt_refs;
    std::string prev_id;
    size_t i = 0;
    for (const auto& t : transitions) {
        auto* node = graph.add_nodes();
        node->set_id(NodeId("bt", i, t.clock.monotonic_ns()));
        node->set_kind(KindFromBt(t.current));
        *node->mutable_clock() = t.clock;
        node->set_label(t.node_name + ":" + t.previous + "->" + t.current);
        node->set_bt_node(t.node_name);
        if (!prev_id.empty()) {
            auto* edge = graph.add_edges();
            edge->set_from_id(prev_id);
            edge->set_to_id(node->id());
            edge->set_relation("temporal");
        }
        prev_id = node->id();
        bt_refs.push_back({node->id(), t.clock.monotonic_ns()});
        if (t.current == "FAILURE") {
            graph.set_failure_source_id(node->id());
        }
        ++i;
    }
    for (const auto& trg : triggers) {
        auto* node = graph.add_nodes();
        node->set_id(trg.evidence_node_id().empty()
                         ? NodeId("trg", i, trg.clock().monotonic_ns())
                         : trg.evidence_node_id());
        node->set_kind(trg.kind() == proto::TRIGGER_ESTOP ||
                               trg.kind() == proto::TRIGGER_TASK_FAILURE
                           ? proto::NODE_FAILURE_SOURCE
                           : proto::NODE_OBSERVATION);
        *node->mutable_clock() = trg.clock();
        node->set_label(trg.detail());
        node->set_channel(trg.channel());
        const int64_t tns = trg.clock().monotonic_ns();
        auto nearest = bt_refs.end();
        int64_t best = kContextWindowNs;
        for (auto it = bt_refs.begin(); it != bt_refs.end(); ++it) {
            const int64_t dt = std::llabs(it->ns - tns);
            if (dt <= best) {
                best = dt;
                nearest = it;
            }
        }
        if (nearest != bt_refs.end()) {
            auto* edge = graph.add_edges();
            edge->set_from_id(nearest->id);
            edge->set_to_id(node->id());
            edge->set_relation("context");
        }
        if (graph.failure_source_id().empty() &&
            (trg.kind() == proto::TRIGGER_ESTOP ||
             trg.kind() == proto::TRIGGER_BT_RECOVER ||
             trg.kind() == proto::TRIGGER_TASK_FAILURE)) {
            graph.set_failure_source_id(node->id());
        }
        ++i;
    }
    return graph;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

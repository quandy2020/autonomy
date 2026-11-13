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
 * @file sidecar.cpp
 * @brief Write sidecar JSON, manifest, overlay, and P0 hazard pointer.
 */

#include "autonomy/system/recorder/sidecar.hpp"

#include <fstream>

#include "nlohmann/json.hpp"

#include "autolink/common/file.hpp"

namespace autonomy {
namespace system {
namespace recorder {
namespace {

nlohmann::json ClockJson(const proto::DualClock& c) {
    return {{"monotonic_ns", c.monotonic_ns()},
            {"wall_utc_ns", c.wall_utc_ns()},
            {"time_source", proto::TimeSource_Name(c.time_source())}};
}

}  // namespace

bool WriteSidecarJson(const std::string& path,
                      const proto::EpisodeSidecar& sidecar) {
    nlohmann::json j;
    j["episode_id"] = sidecar.header().episode_id();
    j["admitted"] = sidecar.header().admitted();
    j["priority"] = proto::Priority_Name(sidecar.header().priority());
    j["bt_root"] = sidecar.header().bt_root();
    j["robot_id"] = sidecar.header().robot_id();
    j["start"] = ClockJson(sidecar.header().start());
    j["end"] = ClockJson(sidecar.header().end());
    j["version"] = {{"policy_version", sidecar.header().version().policy_version()},
                    {"env_version", sidecar.header().version().env_version()},
                    {"git_sha", sidecar.header().version().git_sha()},
                    {"seed", sidecar.header().version().seed()}};
    j["failure_source_id"] = sidecar.graph().failure_source_id();
    j["bt_failure"] = {{"node_name", sidecar.bt_failure().node_name()},
                       {"condition", sidecar.bt_failure().condition()}};
    j["replay"] = {{"valid", sidecar.replay().valid()},
                   {"action_match", sidecar.replay().action_match()},
                   {"bit_exact", sidecar.replay().bit_exact()},
                   {"mode", proto::ReplayMode_Name(sidecar.replay().mode())},
                   {"reason", sidecar.replay().reason()}};
    j["recorded_actions"] = sidecar.recorded_actions_size();
    j["replayed_actions"] = sidecar.replayed_actions_size();
    j["gate"] = {{"passed", sidecar.gate().passed()},
                 {"reason", sidecar.gate().reason()},
                 {"replay_ok", sidecar.gate().replay_ok()},
                 {"safety_ok", sidecar.gate().safety_ok()},
                 {"reroll_ok", sidecar.gate().root_cause_reroll_ok()},
                 {"coverage_ok", sidecar.gate().attribution_coverage_ok()},
                 {"regression_ok", sidecar.gate().history_regression_ok()}};
    nlohmann::json obs = nlohmann::json::array();
    for (const auto& o : sidecar.observations()) {
        nlohmann::json ids = nlohmann::json::array();
        for (const auto& id : o.evidence_node_ids()) {
            ids.push_back(id);
        }
        obs.push_back({{"observation_id", o.observation_id()},
                       {"description", o.description()},
                       {"evidence_node_ids", ids},
                       {"bt_node", o.bt_node()},
                       {"start_monotonic_ns", o.start_monotonic_ns()},
                       {"end_monotonic_ns", o.end_monotonic_ns()}});
    }
    j["observations"] = obs;
    nlohmann::json nodes = nlohmann::json::array();
    for (const auto& n : sidecar.graph().nodes()) {
        nodes.push_back({{"id", n.id()},
                         {"kind", proto::NodeKind_Name(n.kind())},
                         {"label", n.label()},
                         {"bt_node", n.bt_node()},
                         {"clock", ClockJson(n.clock())}});
    }
    j["graph_nodes"] = nodes;
    nlohmann::json edges = nlohmann::json::array();
    for (const auto& e : sidecar.graph().edges()) {
        edges.push_back({{"from_id", e.from_id()},
                         {"to_id", e.to_id()},
                         {"relation", e.relation()}});
    }
    j["graph_edges"] = edges;
    nlohmann::json triggers = nlohmann::json::array();
    for (const auto& t : sidecar.triggers()) {
        triggers.push_back({{"kind", proto::TriggerKind_Name(t.kind())},
                            {"detail", t.detail()},
                            {"evidence_node_id", t.evidence_node_id()},
                            {"channel", t.channel()},
                            {"clock", ClockJson(t.clock())}});
    }
    j["triggers"] = triggers;
    nlohmann::json cards = nlohmann::json::array();
    for (const auto& c : sidecar.root_causes()) {
        cards.push_back({{"hypothesis_id", c.hypothesis_id()},
                         {"evidence_node_id", c.evidence_node_id()},
                         {"statement", c.statement()},
                         {"confidence", c.confidence()},
                         {"reroll_passed", c.reroll_passed()},
                         {"delta_success_set", c.delta_success_set()}});
    }
    j["root_causes"] = cards;
    nlohmann::json safety = nlohmann::json::array();
    for (const auto& s : sidecar.safety_events()) {
        safety.push_back({{"event_id", s.event_id()},
                          {"kind", s.kind()},
                          {"bt_node", s.bt_node()},
                          {"clock", ClockJson(s.clock())}});
    }
    j["safety_events"] = safety;
    return autolink::common::SetStringToASCIIFile(j.dump(2) + "\n", path);
}

bool WriteReplayOverlay(const std::string& path,
                        const proto::ReplayReport& report) {
    nlohmann::json j = {
        {"episode_id", report.episode_id()},
        {"mode", proto::ReplayMode_Name(report.mode())},
        {"bit_exact", report.bit_exact()},
        {"action_match", report.action_match()},
        {"intervention_improved", report.intervention_improved()},
        {"valid", report.valid()},
        {"reason", report.reason()},
    };
    return autolink::common::SetStringToASCIIFile(j.dump(2) + "\n", path);
}

bool WriteManifest(const std::string& path, const proto::EpisodeHeader& header,
                   const UploadDecision& upload) {
    nlohmann::json j = {
        {"episode_id", header.episode_id()},
        {"admitted", header.admitted()},
        {"priority", proto::Priority_Name(upload.priority)},
        {"upload", upload.upload},
        {"expert_demo", upload.expert_demo},
        {"sampled", upload.sampled},
        {"policy", upload.policy},
        {"bt_root", header.bt_root()},
        {"robot_id", header.robot_id()},
    };
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << j.dump(2) << '\n';
    return true;
}

std::vector<proto::Observation> LoadObservationsJsonl(const std::string& path) {
    std::vector<proto::Observation> out;
    std::ifstream in(path);
    if (!in) {
        return out;
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        auto j = nlohmann::json::parse(line, nullptr, false);
        if (j.is_discarded()) {
            continue;
        }
        proto::Observation o;
        o.set_observation_id(j.value("observation_id", ""));
        o.set_description(j.value("description", ""));
        o.set_bt_node(j.value("bt_node", ""));
        o.set_start_monotonic_ns(j.value("start_monotonic_ns", 0));
        o.set_end_monotonic_ns(j.value("end_monotonic_ns", 0));
        if (j.contains("evidence_node_ids")) {
            for (const auto& id : j["evidence_node_ids"]) {
                o.add_evidence_node_ids(id.get<std::string>());
            }
        }
        out.push_back(std::move(o));
    }
    return out;
}

bool WriteHazardPointer(const std::string& log_root,
                        const std::string& episode_id,
                        const std::string& mcap_path,
                        const std::string& sidecar_json) {
    if (log_root.empty() || episode_id.empty()) {
        return false;
    }
    namespace fs = std::filesystem;
    const fs::path dir = fs::path(log_root) / "events" / ("recorder_" + episode_id);
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        return false;
    }
    nlohmann::json j = {{"kind", "recorder_hazard"},
                        {"priority", "P0"},
                        {"episode_id", episode_id},
                        {"mcap", mcap_path},
                        {"sidecar_json", sidecar_json}};
    std::ofstream out(dir / "manifest.json");
    if (!out) {
        return false;
    }
    out << j.dump(2) << '\n';
    return true;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

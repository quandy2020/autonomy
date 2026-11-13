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
 * @file metrics.cpp
 * @brief MPS/MPD, latency percentiles, attribution coverage, PublishGate.
 */

#include "autonomy/system/recorder/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>

#include "nlohmann/json.hpp"

namespace autonomy {
namespace system {
namespace recorder {
namespace {

double Percentile(std::vector<double> values, double p) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double idx = p * static_cast<double>(values.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    if (lo == hi) {
        return values[lo];
    }
    const double w = idx - static_cast<double>(lo);
    return values[lo] * (1.0 - w) + values[hi] * w;
}

}  // namespace

EpisodeMetric MetricFromSidecar(const proto::EpisodeSidecar& sidecar) {
    EpisodeMetric metric;
    metric.success = sidecar.bt_failure().node_name().empty();
    metric.attributed = !sidecar.bt_failure().node_name().empty() ||
                        sidecar.root_causes_size() > 0;
    metric.stupid = !metric.success;
    metric.dangerous = sidecar.safety_events_size() > 0;
    const int64_t dt = sidecar.header().end().monotonic_ns() -
                       sidecar.header().start().monotonic_ns();
    metric.latency_sec = dt > 0 ? static_cast<double>(dt) / 1e9 : 0.0;
    return metric;
}

MetricSnapshot ComputeMetrics(const std::vector<EpisodeMetric>& episodes) {
    MetricSnapshot snap;
    if (episodes.empty()) {
        return snap;
    }
    int stupid = 0;
    int dangerous = 0;
    int attributed = 0;
    int failures = 0;
    std::vector<double> latencies;
    latencies.reserve(episodes.size());
    std::vector<int> success;
    success.reserve(episodes.size());
    for (const auto& e : episodes) {
        stupid += e.stupid ? 1 : 0;
        dangerous += e.dangerous ? 1 : 0;
        if (!e.success) {
            ++failures;
            attributed += e.attributed ? 1 : 0;
        }
        latencies.push_back(e.latency_sec);
        success.push_back(e.success ? 1 : 0);
    }
    const double n = static_cast<double>(episodes.size());
    snap.mps = static_cast<double>(stupid) / n;
    snap.mpd = static_cast<double>(dangerous) / n;
    snap.p95_latency = Percentile(latencies, 0.95);
    snap.p99_latency = Percentile(latencies, 0.99);
    snap.attribution_coverage =
        failures == 0 ? 1.0
                      : static_cast<double>(attributed) /
                            static_cast<double>(failures);

    std::vector<EpisodeMetric> sorted = episodes;
    std::sort(sorted.begin(), sorted.end(),
              [](const EpisodeMetric& a, const EpisodeMetric& b) {
                  return a.latency_sec > b.latency_sec;
              });
    const size_t worst_n = std::max<size_t>(1, sorted.size() / 20);
    int worst_ok = 0;
    for (size_t i = 0; i < worst_n; ++i) {
        worst_ok += sorted[i].success ? 1 : 0;
    }
    snap.worst5_success =
        static_cast<double>(worst_ok) / static_cast<double>(worst_n);
    snap.n = episodes.size();
    return snap;
}

proto::PublishGate EvaluateGate(const MetricSnapshot& metrics,
                                const proto::ReplayReport& replay,
                                bool reroll_ok, bool safety_ok,
                                bool regression_ok, double coverage_floor) {
    proto::PublishGate gate;
    gate.set_history_regression_ok(regression_ok);
    gate.set_root_cause_reroll_ok(reroll_ok);
    gate.set_attribution_coverage_ok(metrics.attribution_coverage >=
                                     coverage_floor);
    gate.set_safety_ok(safety_ok);
    gate.set_replay_ok(replay.valid());
    gate.set_passed(gate.history_regression_ok() &&
                    gate.root_cause_reroll_ok() &&
                    gate.attribution_coverage_ok() && gate.safety_ok() &&
                    gate.replay_ok());
    if (!gate.passed()) {
        std::string reason;
        if (!regression_ok) {
            reason += "regression;";
        }
        if (!reroll_ok) {
            reason += "reroll;";
        }
        if (!gate.attribution_coverage_ok()) {
            reason += "coverage;";
        }
        if (!safety_ok) {
            reason += "safety;";
        }
        if (!replay.valid()) {
            reason += "replay;";
        }
        gate.set_reason(reason);
    }
    return gate;
}

bool AppendEpisodeMetric(const std::string& path, const EpisodeMetric& metric) {
    nlohmann::json j = {{"stupid", metric.stupid},
                        {"dangerous", metric.dangerous},
                        {"attributed", metric.attributed},
                        {"success", metric.success},
                        {"latency_sec", metric.latency_sec}};
    std::ofstream out(path, std::ios::app);
    if (!out) {
        return false;
    }
    out << j.dump() << '\n';
    return true;
}

std::vector<EpisodeMetric> LoadEpisodeMetrics(const std::string& path,
                                              size_t max_n) {
    std::vector<EpisodeMetric> out;
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
        EpisodeMetric m;
        m.stupid = j.value("stupid", false);
        m.dangerous = j.value("dangerous", false);
        m.attributed = j.value("attributed", false);
        m.success = j.value("success", false);
        m.latency_sec = j.value("latency_sec", 0.0);
        out.push_back(m);
    }
    if (out.size() > max_n) {
        out.erase(out.begin(),
                  out.begin() + static_cast<std::ptrdiff_t>(out.size() - max_n));
    }
    return out;
}

bool WriteGateSnapshot(const std::string& path, const proto::PublishGate& gate,
                       const MetricSnapshot& snap,
                       const std::string& episode_id) {
    nlohmann::json j = {
        {"episode_id", episode_id},
        {"publish_allowed", gate.passed()},
        {"gate",
         {{"passed", gate.passed()},
          {"reason", gate.reason()},
          {"replay_ok", gate.replay_ok()},
          {"safety_ok", gate.safety_ok()},
          {"reroll_ok", gate.root_cause_reroll_ok()},
          {"coverage_ok", gate.attribution_coverage_ok()},
          {"regression_ok", gate.history_regression_ok()}}},
        {"fleet",
         {{"n", snap.n},
          {"mps", snap.mps},
          {"mpd", snap.mpd},
          {"attribution_coverage", snap.attribution_coverage},
          {"p95_latency", snap.p95_latency}}},
    };
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << j.dump(2) << '\n';
    return true;
}

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

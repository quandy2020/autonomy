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
 * @file metrics.hpp
 * @brief L5 MPS/MPD, latency percentiles, attribution coverage, and PublishGate.
 */

#include <cstddef>
#include <string>
#include <vector>

#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Per-episode metric row used to accumulate fleet snapshots.
 */
struct EpisodeMetric {
    bool stupid{false};     ///< MPS numerator (avoidable failure).
    bool dangerous{false};  ///< MPD numerator (safety event).
    bool attributed{false}; ///< L3 BT attribution present.
    bool success{false};    ///< Task succeeded.
    double latency_sec{0.0};  ///< End-to-end latency, seconds.
};

/**
 * @brief Aggregated fleet snapshot over N episodes.
 */
struct MetricSnapshot {
    double mps{0.0};                   ///< Mean stupid rate.
    double mpd{0.0};                   ///< Mean dangerous rate.
    double p95_latency{0.0};           ///< 95th-percentile latency.
    double p99_latency{0.0};           ///< 99th-percentile latency.
    double worst5_success{0.0};        ///< Success rate of the worst 5%.
    double attribution_coverage{0.0};  ///< Fraction with L3 attribution.
    size_t n{0};                       ///< Episode count.
};

/**
 * @brief Derive an EpisodeMetric from a sidecar (graph / BT / safety / replay).
 * @param [in] sidecar: Episode sidecar.
 * @return Metric row.
 */
EpisodeMetric MetricFromSidecar(const proto::EpisodeSidecar& sidecar);

/**
 * @brief Aggregate episode rows into a MetricSnapshot.
 * @param [in] episodes: Per-episode rows.
 * @return Snapshot; zeros if empty.
 */
MetricSnapshot ComputeMetrics(const std::vector<EpisodeMetric>& episodes);

/**
 * @brief Evaluate the L5 publish gate (replay + safety + coverage + regression).
 * @param [in] metrics: Fleet snapshot.
 * @param [in] replay: L6 replay report.
 * @param [in] reroll_ok: Hypothesis reroll accepted.
 * @param [in] safety_ok: Hash chain verified / no unlatched hazard.
 * @param [in] regression_ok: No success-set regression.
 * @param [in] coverage_floor: Minimum attribution coverage.
 * @return PublishGate proto.
 */
proto::PublishGate EvaluateGate(const MetricSnapshot& metrics,
                                const proto::ReplayReport& replay,
                                bool reroll_ok, bool safety_ok,
                                bool regression_ok, double coverage_floor);

/**
 * @brief Append one metric row as JSONL.
 * @param [in] path: JSONL path.
 * @param [in] metric: Row to append.
 * @return True on success.
 */
bool AppendEpisodeMetric(const std::string& path, const EpisodeMetric& metric);

/**
 * @brief Load up to max_n metric rows from JSONL (oldest dropped if over cap).
 * @param [in] path: JSONL path.
 * @param [in] max_n: Maximum rows to keep.
 * @return Loaded rows.
 */
std::vector<EpisodeMetric> LoadEpisodeMetrics(const std::string& path,
                                              size_t max_n = 256);

/**
 * @brief Write a gate + snapshot JSON for autoviz / CLI `gate -d`.
 * @param [in] path: Output JSON path.
 * @param [in] gate: PublishGate.
 * @param [in] snap: Metric snapshot.
 * @param [in] episode_id: Episode id.
 * @return True on success.
 */
bool WriteGateSnapshot(const std::string& path, const proto::PublishGate& gate,
                       const MetricSnapshot& snap,
                       const std::string& episode_id);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

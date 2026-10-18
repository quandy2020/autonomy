/*
 * Copyright 2026 The Openbot Authors
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

#include "argus/loop_closure/scan_context_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace argus {
namespace loop_closure {

ScanContextDetector::ScanContextDetector(ScanContextConfig config)
    : config_(std::move(config)) {}

int ScanContextDetector::database_size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(keyframes_.size());
}

void ScanContextDetector::IndexKeyFrame(const KeyFrame& kf) {
    const Descriptor descriptor = BuildDescriptor(kf.cloud);
    const Eigen::VectorXd ring_key = BuildRingKey(descriptor);
    std::lock_guard<std::mutex> lock(mutex_);
    keyframes_.push_back(kf);
    descriptors_.push_back(descriptor);
    ring_keys_.push_back(ring_key);
}

ScanContextDetector::Descriptor ScanContextDetector::BuildDescriptor(
    const sensor::PointCloudXYZI& cloud) const {
    Descriptor descriptor =
        Descriptor::Zero(config_.num_rings, config_.num_sectors);
    Eigen::MatrixXi counts =
        Eigen::MatrixXi::Zero(config_.num_rings, config_.num_sectors);

    const double ring_resolution =
        config_.max_radius_m /
        static_cast<double>(std::max(1, config_.num_rings));
    const double sector_resolution =
        2.0 * M_PI / static_cast<double>(std::max(1, config_.num_sectors));

    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const double x = cloud[i].x;
        const double y = cloud[i].y;
        const double z = cloud[i].z;
        const double range = std::hypot(x, y);
        if (range < 1e-3 || range > config_.max_radius_m) {
            continue;
        }
        int ring = static_cast<int>(range / ring_resolution);
        ring = std::min(ring, config_.num_rings - 1);
        double azimuth = std::atan2(y, x);
        if (azimuth < 0.0) {
            azimuth += 2.0 * M_PI;
        }
        int sector = static_cast<int>(azimuth / sector_resolution);
        sector = std::min(sector, config_.num_sectors - 1);
        // Max height encoding (classic Scan Context).
        descriptor(ring, sector) = std::max(descriptor(ring, sector), z);
        counts(ring, sector) += 1;
    }
    (void)counts;
    return descriptor;
}

Eigen::VectorXd ScanContextDetector::BuildRingKey(
    const Descriptor& descriptor) const {
    Eigen::VectorXd ring_key(descriptor.rows());
    for (int r = 0; r < descriptor.rows(); ++r) {
        ring_key(r) = descriptor.row(r).mean();
    }
    return ring_key;
}

double ScanContextDetector::SectorShiftDistance(const Descriptor& a,
                                                const Descriptor& b,
                                                int* best_shift) const {
    double best_distance = std::numeric_limits<double>::infinity();
    int shift_best = 0;
    const int cols = static_cast<int>(a.cols());
    for (int shift = 0; shift < cols; ++shift) {
        Descriptor b_shifted(b.rows(), b.cols());
        for (int c = 0; c < cols; ++c) {
            b_shifted.col(c) = b.col((c + shift) % cols);
        }
        const double numerator = (a.array() * b_shifted.array()).sum();
        const double denominator =
            a.norm() * b_shifted.norm() +
            std::numeric_limits<double>::epsilon();
        const double distance = 1.0 - (numerator / denominator);  // cosine
        if (distance < best_distance) {
            best_distance = distance;
            shift_best = shift;
        }
    }
    if (best_shift != nullptr) {
        *best_shift = shift_best;
    }
    return best_distance;
}

std::vector<LoopCandidate> ScanContextDetector::Detect(const KeyFrame& kf) {
    const Descriptor descriptor = BuildDescriptor(kf.cloud);
    const Eigen::VectorXd ring_key = BuildRingKey(descriptor);

    std::lock_guard<std::mutex> lock(mutex_);
    keyframes_.push_back(kf);
    descriptors_.push_back(descriptor);
    ring_keys_.push_back(ring_key);

    const int database_size = static_cast<int>(keyframes_.size());
    const int query_index = database_size - 1;
    if (database_size <= config_.exclude_recent_count + 1) {
        return {};
    }

    // Ring-key prefilter.
    struct RingKeyCandidate {
        int index = 0;
        double ring_distance = 0;
    };
    std::vector<RingKeyCandidate> ranked;
    ranked.reserve(static_cast<std::size_t>(database_size));
    const int last_excluded =
        database_size - 1 - config_.exclude_recent_count;
    for (int i = 0; i < last_excluded; ++i) {
        RingKeyCandidate candidate;
        candidate.index = i;
        candidate.ring_distance =
            (ring_keys_[static_cast<std::size_t>(i)] - ring_key).norm();
        ranked.push_back(candidate);
    }
    std::sort(ranked.begin(), ranked.end(),
              [](const RingKeyCandidate& a, const RingKeyCandidate& b) {
                  return a.ring_distance < b.ring_distance;
              });

    const int prefilter_count = std::min(
        config_.max_candidates * 3, static_cast<int>(ranked.size()));
    std::vector<LoopCandidate> out;
    for (int k = 0; k < prefilter_count; ++k) {
        int shift = 0;
        const int match_index =
            ranked[static_cast<std::size_t>(k)].index;
        const double distance = SectorShiftDistance(
            descriptor, descriptors_[static_cast<std::size_t>(match_index)],
            &shift);
        const double score = 1.0 - distance;
        if (score < config_.min_score) {
            continue;
        }
        LoopCandidate candidate;
        candidate.query_id =
            keyframes_[static_cast<std::size_t>(query_index)].id;
        candidate.match_id =
            keyframes_[static_cast<std::size_t>(match_index)].id;
        candidate.score = score;
        // Relative pose from odometry estimates (refined later by ICP / BnB).
        const transform::Rigid3d& T_w_query =
            keyframes_[static_cast<std::size_t>(query_index)].T_w_b;
        const transform::Rigid3d& T_w_match =
            keyframes_[static_cast<std::size_t>(match_index)].T_w_b;
        candidate.T_query_match = T_w_query.inverse() * T_w_match;
        (void)shift;
        out.push_back(candidate);
    }
    std::sort(out.begin(), out.end(),
              [](const LoopCandidate& a, const LoopCandidate& b) {
                  return a.score > b.score;
              });
    if (static_cast<int>(out.size()) > config_.max_candidates) {
        out.resize(static_cast<std::size_t>(config_.max_candidates));
    }
    return out;
}

}  // namespace loop_closure
}  // namespace argus

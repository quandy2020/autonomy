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

/**
 * @file scan_context_detector.hpp
 * @brief Scan Context place recognition (G. Kim & A. Kim, IROS 2018 style).
 */

#ifndef ARGUS_LOOP_CLOSURE_SCAN_CONTEXT_DETECTOR_HPP_
#define ARGUS_LOOP_CLOSURE_SCAN_CONTEXT_DETECTOR_HPP_

#include <mutex>
#include <vector>

#include <Eigen/Core>

#include "argus/common/macros.hpp"
#include "argus/loop_closure/loop_detector_base.hpp"

namespace argus {
namespace loop_closure {

/** @brief Hyper-parameters for ring–sector Scan Context encoding and retrieval.
 */
struct ScanContextConfig {
    int num_rings = 20;
    int num_sectors = 60;
    double max_radius_m = 80.0;
    int max_candidates = 10;
    /** @brief Exclude the most recent N keyframes from candidates. */
    int exclude_recent_count = 30;
    double min_score = 0.5;
};

/**
 * @brief Scan Context loop detector (Kim & Kim, IROS 2018 style).
 *
 * Builds a ring–sector descriptor per keyframe and retrieves place matches
 * via ring-key pre-filtering and sector-shift distance.
 */
class ScanContextDetector : public LoopDetectorBase
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(ScanContextDetector);

    /**
     * @brief Construct with encoding and retrieval settings.
     * @param config Ring/sector grid, radius, and score thresholds.
     */
    explicit ScanContextDetector(ScanContextConfig config = {});

    /**
     * @brief Detect loop candidates for a query keyframe and append it to the
     * DB.
     * @param kf Query keyframe (body cloud and pose).
     * @return Ranked loop candidates above `min_score` (may be empty).
     */
    std::vector<LoopCandidate> Detect(const KeyFrame& kf) override;

    /**
     * @brief Index a keyframe without running retrieval (e.g. prior session).
     * @param kf Keyframe to store in the internal database.
     */
    void IndexKeyFrame(const KeyFrame& kf);

    /**
     * @brief Number of keyframes currently indexed.
     * @return Database size.
     */
    int database_size() const;

private:
    /** @brief Ring–sector height matrix (`num_rings` × `num_sectors`). */
    using Descriptor = Eigen::MatrixXd;

    Descriptor BuildDescriptor(const sensor::PointCloudXYZI& cloud) const;
    Eigen::VectorXd BuildRingKey(const Descriptor& descriptor) const;
    double SectorShiftDistance(const Descriptor& a, const Descriptor& b,
                               int* best_shift) const;

    ScanContextConfig config_;
    mutable std::mutex mutex_;
    std::vector<KeyFrame> keyframes_;
    std::vector<Descriptor> descriptors_;
    std::vector<Eigen::VectorXd> ring_keys_;
};

}  // namespace loop_closure
}  // namespace argus

#endif  // ARGUS_LOOP_CLOSURE_SCAN_CONTEXT_DETECTOR_HPP_

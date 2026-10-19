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
 * @file scan_context.hpp
 * @brief Scan Context++ place index for reloc candidate cropping before BnB.
 */

#ifndef ARGUS_RELOCALIZATION_SCAN_CONTEXT_HPP_
#define ARGUS_RELOCALIZATION_SCAN_CONTEXT_HPP_

#include <mutex>
#include <vector>

#include <Eigen/Core>

#include "argus/common/macros.hpp"
#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/relocalization/candidate.hpp"
#include "argus/relocalization/scan_context/codec.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace relocalization {

/** @brief Scan Context++ prefilter settings ahead of BnB. */
struct PlaceRecognitionConfig {
    bool enable = false; /**< @brief Master switch for place crop. */
    int top_k = 5;       /**< @brief Max candidates after retrieval. */
    double min_score = 0.45; /**< @brief Min cosine similarity. */
    int crop_radius_steps = 3; /**< @brief BnB radius around each seed. */
    bool fallback_full_search =
        true; /**< @brief Full-map BnB if all crops fail. */

    int num_rings = 20;          /**< @brief SC rings. */
    int num_sectors = 60;        /**< @brief SC sectors. */
    double max_radius_m = 80.0;  /**< @brief Polar max range (m). */
    double keyframe_spacing_m =
        5.0; /**< @brief Lattice spacing when sampling prior. */
    int min_points_per_keyframe = 30; /**< @brief Min points per keyframe. */
    int ring_key_prefilter_k =
        50; /**< @brief Ring-key shortlist before sector scoring. */

    bool enable_lateral_search =
        true; /**< @brief SC++ query y-shift search. */
    int lateral_max_shift = 2;     /**< @brief Max |lateral| steps. */
    double lateral_step_m = 1.5;   /**< @brief Lateral step (m). */
};

/**
 * @brief Prior-map Scan Context index (no loop “exclude recent” semantics).
 * @note Retrieval runs only on the relocalization pool thread.
 */
class ScanContextPlaceIndex
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(ScanContextPlaceIndex);

    explicit ScanContextPlaceIndex(PlaceRecognitionConfig config = {});

    /** @brief Drop all indexed keyframes. */
    void Clear();

    /**
     * @brief Spatially sample keyframes from a world-frame prior cloud.
     * @param cloud_w Aggregated map / prior points.
     */
    void BuildFromWorldCloud(const sensor::PointCloudXYZI& cloud_w);

    /**
     * @brief Index an explicit keyframe.
     * @param id Stable id within this index.
     * @param T_w_b Keyframe pose.
     * @param cloud_b Body-frame scan.
     */
    void IndexKeyFrame(int id, const transform::Rigid3d& T_w_b,
                       const sensor::PointCloudXYZI& cloud_b);

    /**
     * @brief Retrieve Top-K place candidates for a body-frame query.
     * @param query_cloud_b Query in body frame.
     * @return Ranked candidates (may be empty).
     */
    std::vector<PlaceCandidate> Retrieve(
        const sensor::PointCloudXYZI& query_cloud_b) const;

    /** @return Number of indexed keyframes. */
    int database_size() const;

    /** @return Current config. */
    const PlaceRecognitionConfig& config() const {
        return config_;
    }

    /**
     * @brief Replace config (does not rebuild the index).
     * @param config New settings.
     */
    void set_config(PlaceRecognitionConfig config) {
        config_ = std::move(config);
    }

private:
    struct Entry {
        int id = 0;
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        sc::Descriptor descriptor;
        Eigen::VectorXd ring_key;
    };

    sc::GridConfig Grid() const;

    PlaceRecognitionConfig config_;
    mutable std::mutex mutex_;
    std::vector<Entry> entries_;
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_SCAN_CONTEXT_HPP_

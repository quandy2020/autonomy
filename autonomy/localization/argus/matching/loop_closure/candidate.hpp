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
 * @file candidate.hpp
 * @brief Place-candidate type and pluggable retrieval interface for reloc.
 */

#ifndef ARGUS_RELOCALIZATION_CANDIDATE_HPP_
#define ARGUS_RELOCALIZATION_CANDIDATE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "argus/common/macros.hpp"
#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace relocalization {

/** @brief One place hypothesis used to seed cropped BnB. */
struct PlaceCandidate {
    int keyframe_id = -1;          /**< @brief Index in the place database. */
    double score = 0.0;            /**< @brief Similarity in [0, 1]. */
    transform::Rigid3d T_w_b =
        transform::Rigid3d::Identity(); /**< @brief World pose seed. */
    double yaw_shift_rad = 0.0;    /**< @brief Sector-align yaw (metadata). */
    double lateral_shift_m = 0.0;  /**< @brief SC++ lateral shift (metadata). */
};

/**
 * @brief Abstract Top-K place retrieval (e.g. future visual prefilter).
 * @note Invoked only on the relocalization thread pool — never ESIKF.
 */
class PlaceCandidateSource
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(PlaceCandidateSource);

    virtual ~PlaceCandidateSource() = default;

    /** @return Short modality tag (e.g. `"visual"`). */
    virtual std::string name() const = 0;

    /** @return Indexed place count (`0` ⇒ inactive). */
    virtual int database_size() const = 0;

    /**
     * @brief Retrieve ranked place hypotheses for a body-frame query.
     * @param query_cloud_b Query scan in body frame.
     * @return Candidates (may be empty).
     */
    virtual std::vector<PlaceCandidate> Retrieve(
        const sensor::PointCloudXYZI& query_cloud_b) const = 0;
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_CANDIDATE_HPP_

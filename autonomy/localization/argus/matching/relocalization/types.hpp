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
 * @file types.hpp
 * @brief Relocalization request / result types and search interface.
 */

#ifndef ARGUS_RELOCALIZATION_TYPES_HPP_
#define ARGUS_RELOCALIZATION_TYPES_HPP_

#include <atomic>
#include <cstdint>

#include "argus/common/math/transform/transform.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace relocalization {

/** @brief Best pose hypothesis from a search. */
struct RelocPose {
    transform::Rigid3d T_w_b =
        transform::Rigid3d::Identity(); /**< @brief Body in world. */
    double score = 0.0;                 /**< @brief Occupancy hit ratio. */
};

/** @brief One relocalization query. */
struct RelocRequest {
    uint64_t id = 0; /**< @brief Caller-assigned or manager-assigned id. */
    sensor::PointCloudXYZI query_cloud; /**< @brief Body-frame query scan. */
    transform::Rigid3d initial_guess =
        transform::Rigid3d::Identity(); /**< @brief Optional pose seed. */
    bool has_initial_guess = false;     /**< @brief Whether `initial_guess` is set. */
};

/** @brief Outcome of one Search. */
struct RelocResult {
    uint64_t request_id = 0; /**< @brief Echo of RelocRequest::id. */
    bool success = false;    /**< @brief Score ≥ threshold. */
    bool cancelled = false;  /**< @brief Aborted via cancel flag. */
    RelocPose best;          /**< @brief Best pose found (even if !success). */
};

/** @brief Pose error vs ground truth (unit / offline eval). */
struct RelocEvalMetrics {
    double translation_error_m = 0.0; /**< @brief ‖t‖ of T_gt⁻¹ T_est. */
    double rotation_error_rad = 0.0;  /**< @brief Angle of relative rotation. */
    bool within_tolerance = false;    /**< @brief Both errors under thresholds. */
};

/**
 * @brief Compare estimated \(T_{wb}\) to ground truth.
 * @param T_est Estimated body pose in world.
 * @param T_gt Ground-truth body pose in world.
 * @param trans_tol_m Translation threshold for `within_tolerance`.
 * @param rot_tol_rad Rotation threshold for `within_tolerance`.
 * @return Error metrics.
 */
inline RelocEvalMetrics EvaluateAgainstGroundTruth(
    const transform::Rigid3d& T_est, const transform::Rigid3d& T_gt,
    double trans_tol_m = 0.5, double rot_tol_rad = 0.2) {
    RelocEvalMetrics m;
    const transform::Rigid3d err = T_gt.inverse() * T_est;
    m.translation_error_m = err.translation().norm();
    m.rotation_error_rad = transform::GetAngle(err);
    m.within_tolerance =
        (m.translation_error_m <= trans_tol_m) &&
        (m.rotation_error_rad <= rot_tol_rad);
    return m;
}

/**
 * @brief Blocking pose search interface.
 * @note Implementors must honor `cancel` and never run on the ESIKF thread.
 */
class RelocalizationBase
{
public:
    virtual ~RelocalizationBase() = default;

    /**
     * @brief Blocking search.
     * @param request Query cloud and optional guess.
     * @param cancel Cooperative cancel flag (may be null).
     * @return Best pose / score / status.
     */
    virtual RelocResult Search(const RelocRequest& request,
                               std::atomic<bool>* cancel) = 0;
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_TYPES_HPP_

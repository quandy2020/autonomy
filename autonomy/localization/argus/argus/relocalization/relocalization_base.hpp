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
 * @file relocalization_base.hpp
 * @brief Relocalization request / result types and search interface.
 */

#ifndef ARGUS_RELOCALIZATION_RELOCALIZATION_BASE_HPP_
#define ARGUS_RELOCALIZATION_RELOCALIZATION_BASE_HPP_

#include <atomic>
#include <cstdint>

#include "argus/common/math/transform/transform.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace relocalization {

struct RelocPose {
    transform::Rigid3d T_w_b = transform::Rigid3d::Identity();
    double score = 0.0;
};

struct RelocRequest {
    uint64_t id = 0;
    /** @brief Query cloud in body frame. */
    sensor::PointCloudXYZI query_cloud;
    transform::Rigid3d initial_guess = transform::Rigid3d::Identity();
    bool has_initial_guess = false;
};

struct RelocResult {
    uint64_t request_id = 0;
    bool success = false;
    bool cancelled = false;
    RelocPose best;
};

/** @brief Pose error vs ground-truth (for offline / unit evaluation). */
struct RelocEvalMetrics {
    double translation_error_m = 0.0;
    double rotation_error_rad = 0.0;
    bool within_tolerance = false;
};

/**
 * @brief Compare estimated \(T_{w b}\) to ground truth.
 * @param trans_tol_m  Translation threshold for `within_tolerance`.
 * @param rot_tol_rad  Rotation threshold for `within_tolerance`.
 */
inline RelocEvalMetrics EvaluateAgainstGroundTruth(
    const transform::Rigid3d& T_est, const transform::Rigid3d& T_gt,
    double trans_tol_m = 0.5, double rot_tol_rad = 0.2) {
    RelocEvalMetrics m;
    const transform::Rigid3d err = T_gt.inverse() * T_est;
    m.translation_error_m = err.translation().norm();
    m.rotation_error_rad = transform::GetAngle(err);
    m.within_tolerance =
        (m.translation_error_m <= trans_tol_m) && (m.rotation_error_rad <= rot_tol_rad);
    return m;
}

class RelocalizationBase
{
public:
    virtual ~RelocalizationBase() = default;

    /**
     * @brief Blocking search; check `cancel` frequently and abort if set.
     * @note Call only from the relocalization thread pool — never ESIKF.
     */
    virtual RelocResult Search(const RelocRequest& request,
                               std::atomic<bool>* cancel) = 0;
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_RELOCALIZATION_BASE_HPP_

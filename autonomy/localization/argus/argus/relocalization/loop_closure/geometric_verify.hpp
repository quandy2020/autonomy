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
 * @file geometric_verify.hpp
 * @brief Point-to-point ICP refine + fitness gate for loop candidates.
 */

#ifndef ARGUS_RELOCALIZATION_LOOP_GEOMETRIC_VERIFY_HPP_
#define ARGUS_RELOCALIZATION_LOOP_GEOMETRIC_VERIFY_HPP_

#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace loop_closure {

/** @brief Why geometric verification rejected a candidate. */
enum class GeomFailReason {
    kOk = 0,
    kEmptyCloud,
    kTooFewPoints,
    kTooFewCorrespondences,
    kFitness,
    kTranslation,
    kYaw,
    kInlierRatio,
};

/** @brief ICP refine / accept gates for Scan-Context seeds. */
struct GeometricVerifyConfig {
    bool enable = true;
    int max_iterations = 12;
    /** @brief Max mean NN distance after refine (m). */
    double max_fitness_m = 0.6;
    /** @brief Reject if |t| of refined \(T_{q\leftarrow m}\) exceeds this. */
    double max_translation_m = 8.0;
    /** @brief Reject if |yaw| exceeds this (rad). */
    double max_yaw_rad = 0.9;
    /** @brief Subsample stride for speed (1 = all). */
    int subsample_stride = 4;
    /** @brief Min inlier correspondences. */
    int min_correspondences = 40;
    /** @brief Final NN accept radius for correspondences (m). */
    double max_corr_dist_m = 2.0;
    /**
     * @brief Min inlier ratio (#corr / #match samples). 0 disables.
     */
    double min_inlier_ratio = 0.25;
    /**
     * @brief Half-range of XY seed lattice around SC seed (steps each axis).
     *        0 = only the provided seed.
     */
    int seed_xy_steps = 1;
    /** @brief XY lattice step (m). */
    double seed_xy_step_m = 1.0;
    /**
     * @brief Half-range of yaw seed lattice (steps). 0 = no extra yaw seeds.
     */
    int seed_yaw_steps = 1;
    /** @brief Yaw lattice step (rad). */
    double seed_yaw_step_rad = 0.15;
};

/** @brief Outcome of geometric verification. */
struct GeometricVerifyResult {
    bool ok = false;
    GeomFailReason fail_reason = GeomFailReason::kOk;
    transform::Rigid3d T_query_match = transform::Rigid3d::Identity();
    double fitness_m = 1e9;
    int correspondences = 0;
    double inlier_ratio = 0.0;
};

/**
 * @brief Refine \(T_{q\leftarrow m}\) so \(p_q \approx T\, p_m\) (body frames).
 * @param cloud_query Query body cloud.
 * @param cloud_match Match body cloud.
 * @param T_seed SC yaw/lateral seed.
 * @param config ICP / gate options.
 * @return Refined transform when `ok`; otherwise reject.
 */
GeometricVerifyResult VerifyAndRefineLoop(
    const sensor::PointCloudXYZI& cloud_query,
    const sensor::PointCloudXYZI& cloud_match,
    const transform::Rigid3d& T_seed, const GeometricVerifyConfig& config);

}  // namespace loop_closure
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_LOOP_GEOMETRIC_VERIFY_HPP_

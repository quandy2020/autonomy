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
 * @file codec.hpp
 * @brief Shared Scan Context encoding / distance helpers (max-z ring–sector).
 */

#ifndef ARGUS_RELOCALIZATION_SCAN_CONTEXT_CODEC_HPP_
#define ARGUS_RELOCALIZATION_SCAN_CONTEXT_CODEC_HPP_

#include <Eigen/Core>

#include "argus/sensor/point_types.hpp"

namespace argus {
namespace relocalization {
namespace sc {

/** @brief Polar grid used by Scan Context descriptors. */
struct GridConfig {
    int num_rings = 20;
    int num_sectors = 60;
    double max_radius_m = 80.0;
};

/** @brief Ring × sector max-height matrix. */
using Descriptor = Eigen::MatrixXd;

/**
 * @brief Encode a body-frame cloud as a Scan Context descriptor.
 * @param cloud Body-frame points.
 * @param grid Polar binning.
 * @return Descriptor (zeros if empty / out of range).
 */
Descriptor BuildDescriptor(const sensor::PointCloudXYZI& cloud,
                           const GridConfig& grid);

/**
 * @brief Row-wise mean ring key for coarse retrieval.
 * @param descriptor Scan Context matrix.
 * @return Vector of length `num_rings`.
 */
Eigen::VectorXd BuildRingKey(const Descriptor& descriptor);

/**
 * @brief Cosine distance under best sector (column) alignment.
 * @param query Query descriptor.
 * @param match Database descriptor.
 * @param[out] best_shift Optional best column shift in \f$[0, N_{\mathrm{sec}})\f$.
 * @return Distance in \f$[0, 2]\f$ (lower is better); score ≈ `1 - distance`.
 */
double SectorShiftDistance(const Descriptor& query, const Descriptor& match,
                           int* best_shift);

/**
 * @brief Translate cloud along body +Y (SC++ lateral search).
 * @param cloud Input cloud.
 * @param dy Lateral shift (m).
 * @return Shifted copy (or `cloud` if `|dy|` ≈ 0).
 */
sensor::PointCloudXYZI TranslateCloudY(const sensor::PointCloudXYZI& cloud,
                                       double dy);

}  // namespace sc
}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_SCAN_CONTEXT_CODEC_HPP_

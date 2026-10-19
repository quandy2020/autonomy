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

#include "argus/relocalization/scan_context/codec.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace argus {
namespace relocalization {
namespace sc {

Descriptor BuildDescriptor(const sensor::PointCloudXYZI& cloud,
                           const GridConfig& grid) {
    Descriptor descriptor =
        Descriptor::Zero(grid.num_rings, grid.num_sectors);
    const double ring_resolution =
        grid.max_radius_m /
        static_cast<double>(std::max(1, grid.num_rings));
    const double sector_resolution =
        2.0 * M_PI / static_cast<double>(std::max(1, grid.num_sectors));

    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const double x = cloud[i].x;
        const double y = cloud[i].y;
        const double z = cloud[i].z;
        const double range = std::hypot(x, y);
        if (range < 1e-3 || range > grid.max_radius_m) {
            continue;
        }
        int ring = static_cast<int>(range / ring_resolution);
        ring = std::min(ring, grid.num_rings - 1);
        double azimuth = std::atan2(y, x);
        if (azimuth < 0.0) {
            azimuth += 2.0 * M_PI;
        }
        int sector = static_cast<int>(azimuth / sector_resolution);
        sector = std::min(sector, grid.num_sectors - 1);
        descriptor(ring, sector) = std::max(descriptor(ring, sector), z);
    }
    return descriptor;
}

Eigen::VectorXd BuildRingKey(const Descriptor& descriptor) {
    Eigen::VectorXd ring_key(descriptor.rows());
    for (int r = 0; r < descriptor.rows(); ++r) {
        ring_key(r) = descriptor.row(r).mean();
    }
    return ring_key;
}

double SectorShiftDistance(const Descriptor& query, const Descriptor& match,
                           int* best_shift) {
    double best_distance = std::numeric_limits<double>::infinity();
    int shift_best = 0;
    const int cols = static_cast<int>(query.cols());
    for (int shift = 0; shift < cols; ++shift) {
        Descriptor match_shifted(match.rows(), match.cols());
        for (int c = 0; c < cols; ++c) {
            match_shifted.col(c) = match.col((c + shift) % cols);
        }
        const double numerator = (query.array() * match_shifted.array()).sum();
        const double denominator =
            query.norm() * match_shifted.norm() +
            std::numeric_limits<double>::epsilon();
        const double distance = 1.0 - (numerator / denominator);
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

sensor::PointCloudXYZI TranslateCloudY(const sensor::PointCloudXYZI& cloud,
                                       double dy) {
    if (std::abs(dy) < 1e-12) {
        return cloud;
    }
    sensor::PointCloudXYZI out = cloud;
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i].y = static_cast<float>(static_cast<double>(out[i].y) + dy);
    }
    return out;
}

}  // namespace sc
}  // namespace relocalization
}  // namespace argus

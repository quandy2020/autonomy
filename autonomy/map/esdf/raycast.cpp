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

/**
 * @file raycast.cpp
 * @brief Amanatides–Woo voxel traversal (implementation).
 */

#include "autonomy/map/esdf/raycast.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace autonomy {
namespace map {
namespace esdf {
namespace {

int Sign(int value) { return value == 0 ? 0 : (value < 0 ? -1 : 1); }

double PositiveModulo(double value, double modulus) {
    return std::fmod(std::fmod(value, modulus) + modulus, modulus);
}

double ParametricDistanceToNextVoxel(double origin_coordinate,
                                     double direction_coordinate) {
    if (direction_coordinate < 0.0) {
        return ParametricDistanceToNextVoxel(-origin_coordinate,
                                             -direction_coordinate);
    }
    origin_coordinate = PositiveModulo(origin_coordinate, 1.0);
    return (1.0 - origin_coordinate) / direction_coordinate;
}

}  // namespace

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
             const Eigen::Vector3d& min_bound, const Eigen::Vector3d& max_bound,
             std::vector<Eigen::Vector3d>* output) {
    int voxel_x = static_cast<int>(std::floor(start.x()));
    int voxel_y = static_cast<int>(std::floor(start.y()));
    int voxel_z = static_cast<int>(std::floor(start.z()));
    const int end_voxel_x = static_cast<int>(std::floor(end.x()));
    const int end_voxel_y = static_cast<int>(std::floor(end.y()));
    const int end_voxel_z = static_cast<int>(std::floor(end.z()));
    const Eigen::Vector3d direction = end - start;
    const double max_squared_length = direction.squaredNorm();

    const double voxel_span_x = static_cast<double>(end_voxel_x - voxel_x);
    const double voxel_span_y = static_cast<double>(end_voxel_y - voxel_y);
    const double voxel_span_z = static_cast<double>(end_voxel_z - voxel_z);

    const int step_x = Sign(static_cast<int>(voxel_span_x));
    const int step_y = Sign(static_cast<int>(voxel_span_y));
    const int step_z = Sign(static_cast<int>(voxel_span_z));

    const double infinity = std::numeric_limits<double>::infinity();
    double parametric_max_x =
        step_x == 0 ? infinity
                    : ParametricDistanceToNextVoxel(start.x(), voxel_span_x);
    double parametric_max_y =
        step_y == 0 ? infinity
                    : ParametricDistanceToNextVoxel(start.y(), voxel_span_y);
    double parametric_max_z =
        step_z == 0 ? infinity
                    : ParametricDistanceToNextVoxel(start.z(), voxel_span_z);

    const double parametric_step_x =
        step_x == 0 ? infinity : static_cast<double>(step_x) / voxel_span_x;
    const double parametric_step_y =
        step_y == 0 ? infinity : static_cast<double>(step_y) / voxel_span_y;
    const double parametric_step_z =
        step_z == 0 ? infinity : static_cast<double>(step_z) / voxel_span_z;

    output->clear();
    if (step_x == 0 && step_y == 0 && step_z == 0) {
        return;
    }

    while (true) {
        if (voxel_x >= min_bound.x() && voxel_x < max_bound.x() &&
            voxel_y >= min_bound.y() && voxel_y < max_bound.y() &&
            voxel_z >= min_bound.z() && voxel_z < max_bound.z()) {
            output->emplace_back(voxel_x, voxel_y, voxel_z);
            if ((Eigen::Vector3d(voxel_x, voxel_y, voxel_z) - start).squaredNorm() >
                max_squared_length) {
                return;
            }
            if (output->size() > 1500) {
                throw std::out_of_range("Raycast exceeded 1500 voxels");
            }
        }
        if (voxel_x == end_voxel_x && voxel_y == end_voxel_y &&
            voxel_z == end_voxel_z) {
            break;
        }
        if (parametric_max_x < parametric_max_y) {
            if (parametric_max_x < parametric_max_z) {
                voxel_x += step_x;
                parametric_max_x += parametric_step_x;
            } else {
                voxel_z += step_z;
                parametric_max_z += parametric_step_z;
            }
        } else if (parametric_max_y < parametric_max_z) {
            voxel_y += step_y;
            parametric_max_y += parametric_step_y;
        } else {
            voxel_z += step_z;
            parametric_max_z += parametric_step_z;
        }
    }
}

}  // namespace esdf
}  // namespace map
}  // namespace autonomy

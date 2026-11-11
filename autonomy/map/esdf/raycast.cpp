/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "autonomy/map/esdf/raycast.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace autonomy {
namespace map {
namespace esdf {
namespace {

int Signum(int x) { return x == 0 ? 0 : (x < 0 ? -1 : 1); }

double ModPositive(double value, double modulus) {
    return std::fmod(std::fmod(value, modulus) + modulus, modulus);
}

double IntBound(double s, double ds) {
    if (ds < 0.0) {
        return IntBound(-s, -ds);
    }
    s = ModPositive(s, 1.0);
    return (1.0 - s) / ds;
}

}  // namespace

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
             const Eigen::Vector3d& min, const Eigen::Vector3d& max,
             std::vector<Eigen::Vector3d>* output) {
    int x = static_cast<int>(std::floor(start.x()));
    int y = static_cast<int>(std::floor(start.y()));
    int z = static_cast<int>(std::floor(start.z()));
    const int end_x = static_cast<int>(std::floor(end.x()));
    const int end_y = static_cast<int>(std::floor(end.y()));
    const int end_z = static_cast<int>(std::floor(end.z()));
    const Eigen::Vector3d direction = end - start;
    const double max_dist = direction.squaredNorm();

    const double dx = static_cast<double>(end_x - x);
    const double dy = static_cast<double>(end_y - y);
    const double dz = static_cast<double>(end_z - z);

    const int step_x = Signum(static_cast<int>(dx));
    const int step_y = Signum(static_cast<int>(dy));
    const int step_z = Signum(static_cast<int>(dz));

    const double inf = std::numeric_limits<double>::infinity();
    double t_max_x = step_x == 0 ? inf : IntBound(start.x(), dx);
    double t_max_y = step_y == 0 ? inf : IntBound(start.y(), dy);
    double t_max_z = step_z == 0 ? inf : IntBound(start.z(), dz);

    const double t_delta_x = step_x == 0 ? inf : static_cast<double>(step_x) / dx;
    const double t_delta_y = step_y == 0 ? inf : static_cast<double>(step_y) / dy;
    const double t_delta_z = step_z == 0 ? inf : static_cast<double>(step_z) / dz;

    output->clear();
    if (step_x == 0 && step_y == 0 && step_z == 0) {
        return;
    }

    while (true) {
        if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() &&
            z >= min.z() && z < max.z()) {
            output->emplace_back(x, y, z);
            if ((Eigen::Vector3d(x, y, z) - start).squaredNorm() > max_dist) {
                return;
            }
            if (output->size() > 1500) {
                throw std::out_of_range("Raycast exceeded 1500 voxels");
            }
        }
        if (x == end_x && y == end_y && z == end_z) {
            break;
        }
        if (t_max_x < t_max_y) {
            if (t_max_x < t_max_z) {
                x += step_x;
                t_max_x += t_delta_x;
            } else {
                z += step_z;
                t_max_z += t_delta_z;
            }
        } else if (t_max_y < t_max_z) {
            y += step_y;
            t_max_y += t_delta_y;
        } else {
            z += step_z;
            t_max_z += t_delta_z;
        }
    }
}

}  // namespace esdf
}  // namespace map
}  // namespace autonomy

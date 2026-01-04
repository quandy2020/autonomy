/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <Eigen/Eigen>
#include <vector>

namespace autonomy {
namespace planning {
namespace planner {
namespace egvg {

// Sign function used by the ray casting algorithm.
int signum(int x);

// Modulo helper that always returns a positive result in [0, modulus).
double mod(double value, double modulus);

// Find the smallest positive t such that s + t * ds is an integer.
double intbound(double s, double ds);

// Raycast from start to end within [min, max), returning grid points in a
// preallocated buffer.
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
             const Eigen::Vector3d& min, const Eigen::Vector3d& max,
             int& output_points_cnt, Eigen::Vector3d* output);

// Raycast from start to end within [min, max), returning grid points in a
// vector.
void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
             const Eigen::Vector3d& min, const Eigen::Vector3d& max,
             std::vector<Eigen::Vector3d>* output);

class RayCaster
{
public:
    RayCaster() = default;
    ~RayCaster() = default;

    bool setInput(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d&
            end /* , const Eigen::Vector3d& min, const Eigen::Vector3d& max */);

    bool step(Eigen::Vector3d& ray_pt, bool verbose = false);

private:
    Eigen::Vector3d start_;
    Eigen::Vector3d end_;
    Eigen::Vector3d direction_;
    Eigen::Vector3d min_;
    Eigen::Vector3d max_;
    int x_;
    int y_;
    int z_;
    int endX_;
    int endY_;
    int endZ_;
    double maxDist_;
    double dx_;
    double dy_;
    double dz_;
    int stepX_;
    int stepY_;
    int stepZ_;
    double tMaxX_;
    double tMaxY_;
    double tMaxZ_;
    double tDeltaX_;
    double tDeltaY_;
    double tDeltaZ_;
    double dist_;

    int step_num_;
};

}  // namespace egvg
}  // namespace planner
}  // namespace planning
}  // namespace autonomy
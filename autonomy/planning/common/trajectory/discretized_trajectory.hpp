/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file discretized_trajectory.hpp
 **/

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace autonomy {
namespace planning {

// Path point structure
struct PathPoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double theta = 0.0;
    double kappa = 0.0;    // Curvature
    double s = 0.0;        // Arc length
    double dkappa = 0.0;   // Derivative of curvature
    double ddkappa = 0.0;  // Second derivative of curvature
};

// Trajectory point structure
struct TrajectoryPoint {
    PathPoint path_point_;
    double v_ = 0.0;              // Velocity
    double a_ = 0.0;              // Acceleration
    double relative_time_ = 0.0;  // Relative time from start

    // Accessors
    double relative_time() const {
        return relative_time_;
    }
    double v() const {
        return v_;
    }
    double a() const {
        return a_;
    }
    const PathPoint& path_point() const {
        return path_point_;
    }
    PathPoint& path_point() {
        return path_point_;
    }
};

class DiscretizedTrajectory
{
public:
    DiscretizedTrajectory() = default;
    ~DiscretizedTrajectory() = default;

    // Iterator support for range-based for loops
    using const_iterator = std::vector<TrajectoryPoint>::const_iterator;
    const_iterator begin() const {
        return trajectory_points_.begin();
    }
    const_iterator end() const {
        return trajectory_points_.end();
    }

    // Get number of points
    size_t NumOfPoints() const {
        return trajectory_points_.size();
    }

    // Get trajectory point at index
    const TrajectoryPoint& TrajectoryPointAt(uint32_t index) const {
        return trajectory_points_.at(index);
    }

    // Add trajectory point
    void AppendTrajectoryPoint(const TrajectoryPoint& point) {
        trajectory_points_.push_back(point);
    }

private:
    std::vector<TrajectoryPoint> trajectory_points_;
};

}  // namespace planning
}  // namespace autonomy

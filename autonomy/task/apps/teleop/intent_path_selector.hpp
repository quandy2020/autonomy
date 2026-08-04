/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include <optional>
#include <vector>

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy::task::teleop {

struct PathCandidate {
    double end_dir_deg{0.0};
    automsgs::msgs::nav_msgs::Path path;
};

class IntentPathSelector
{
public:
    void GenerateDefaultLibrary(int num_dirs, int num_lengths, double max_range,
                                double ds);

    std::optional<automsgs::msgs::nav_msgs::Path> Select(
        const map::costmap_2d::Costmap2D& costmap, double joy_dir_deg,
        double joy_speed) const;

    const std::vector<PathCandidate>& candidates() const {
        return candidates_;
    }

private:
    static int CountLethalHits(const map::costmap_2d::Costmap2D& map,
                               const automsgs::msgs::nav_msgs::Path& path);
    static double WrapDeg(double deg);

    std::vector<PathCandidate> candidates_;
    int point_per_path_thre_{2};
    double dir_weight_{0.02};
};

}  // namespace autonomy::task::teleop

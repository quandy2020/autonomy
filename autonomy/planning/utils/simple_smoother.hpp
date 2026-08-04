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

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/smoother_interface.hpp"
#include "autonomy/planning/proto/simple_smoother.pb.h"

namespace autonomy {
namespace planning {
namespace utils {

/**
 * @class SimpleSmoother
 * @brief A path smoother implementation
 */
class SimpleSmoother : public common::Smoother
{
public:
    /** Default constructor for plugin registration. */
    SimpleSmoother() = default;

    SimpleSmoother(
        std::string name,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper,
        const proto::SimpleSmootherOptions& options = {});

    ~SimpleSmoother() override;

    void ApplyOptions(const proto::SimpleSmootherOptions& options);

    bool Smooth(automsgs::msgs::nav_msgs::Path& path,
                const std::chrono::milliseconds& max_time) override;

protected:
    void SmoothImpl(automsgs::msgs::nav_msgs::Path& path, bool& reversing_segment,
                    const map::costmap_2d::Costmap2D* costmap,
                    const double& max_time);

    inline double GetFieldByDim(const automsgs::msgs::geometry_msgs::PoseStamped& msg,
                                const unsigned int& dim);

    inline void SetFieldByDim(automsgs::msgs::geometry_msgs::PoseStamped& msg,
                              const unsigned int dim, const double& value);

    void ApplyDefaultTuning();

    double tolerance_{0.0};
    double data_w_{0.0};
    double smooth_w_{0.0};
    int max_its_{0};
    int refinement_ctr_{0};
    int refinement_num_{0};
    bool do_refinement_{false};
    bool enforce_path_inversion_{false};
};

}  // namespace utils
}  // namespace planning
}  // namespace autonomy

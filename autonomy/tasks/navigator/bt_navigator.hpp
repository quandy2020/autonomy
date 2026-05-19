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

#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {

class BtNavigator
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(BtNavigator)

    explicit BtNavigator(const autonomy::tasks::proto::TaskOptions& options);

    ~BtNavigator();

protected:
    std::vector<std::shared_ptr<common::NavigatorBase>> navigators_;
    common::NavigatorMuxer plugin_muxer_;
    std::shared_ptr<control::utils::OdomSmoother> odom_smoother_;
    std::string robot_frame_;
    std::string global_frame_;
    double transform_tolerance_{0.1};
    double filter_duration_{0.3};
    double local_survival_timeout_{120.0};
    std::string odom_topic_;
    std::shared_ptr<transform::Buffer> tf_;
    autonomy::tasks::proto::TaskOptions options_;
};

}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy

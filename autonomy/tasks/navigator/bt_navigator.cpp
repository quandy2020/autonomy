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

#include "autonomy/tasks/navigator/bt_navigator.hpp"

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {

namespace {

constexpr double kDefaultTransformTolerance = 0.1;
constexpr double kDefaultFilterDuration = 0.3;
const char kDefaultGlobalFrame[] = "map";
const char kDefaultRobotFrame[] = "base_link";
const char kDefaultOdomTopic[] = "odom";

}  // namespace

BtNavigator::BtNavigator(const autonomy::tasks::proto::TaskOptions& options)
    : options_(options) {
    global_frame_ = options.global_frame().empty() ? kDefaultGlobalFrame
                                                   : options.global_frame();
    robot_frame_ = options.robot_base_frame().empty()
                       ? kDefaultRobotFrame
                       : options.robot_base_frame();
    odom_topic_ =
        options.odom_topic().empty() ? kDefaultOdomTopic : options.odom_topic();
    filter_duration_ = options.filter_duration() > 0.0
                           ? options.filter_duration()
                           : kDefaultFilterDuration;
    if (options.local_survival_timeout() > 0.0) {
        local_survival_timeout_ = options.local_survival_timeout();
    }
    tf_ = std::shared_ptr<transform::Buffer>(transform::Buffer::Instance(),
                                             [](transform::Buffer*) {});
    AINFO << "BtNavigator created: global=" << global_frame_
          << ", robot=" << robot_frame_ << ", odom_topic=" << odom_topic_;
}

BtNavigator::~BtNavigator() {}

}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy

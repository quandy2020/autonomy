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
 *
 * Design aligned with nav2_bt_navigator/src/bt_navigator.cpp.
 */

#include "autonomy/tasks/navigator/bt_navigator.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autolink/common/log.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {

namespace {

constexpr double kDefaultTransformTolerance = 0.1;
constexpr double kDefaultFilterDuration = 0.3;
const char kDefaultGlobalFrame[] = "map";
const char kDefaultRobotFrame[] = "base_link";
const char kDefaultOdomTopic[] = "odom";

// 根据 navigator 名称从 TaskOptions 取 plugin 类型；若未配置则用默认约定 "autonomy_tasks/<id>"
std::string GetPluginType(const std::string& navigator_id, const autonomy::tasks::proto::TaskOptions& options) {
    if (navigator_id == "navigate_to_pose" && options.has_navigate_to_pose() &&
        !options.navigate_to_pose().plugin().empty()) {
        return options.navigate_to_pose().plugin();
    }
    if (navigator_id == "navigate_through_poses" && options.has_navigate_through_poses() &&
        !options.navigate_through_poses().plugin().empty()) {
        return options.navigate_through_poses().plugin();
    }
    if (navigator_id == "navigate_to_docking" && options.has_navigate_to_docking() &&
        !options.navigate_to_docking().plugin().empty()) {
        return options.navigate_to_docking().plugin();
    }
    if (navigator_id == "track_to_target" && options.has_track_to_target() &&
        !options.track_to_target().plugin().empty()) {
        return options.track_to_target().plugin();
    }
    if (navigator_id == "explore_to_anywhere" && options.has_explore_to_anywhere() &&
        !options.explore_to_anywhere().plugin().empty()) {
        return options.explore_to_anywhere().plugin();
    }
    return std::string("autonomy_tasks/") + navigator_id;
}

}  // namespace

namespace {

constexpr char kNavigatorBaseClass[] = "autonomy::tasks::common::NavigatorBase";

}  // namespace

BtNavigator::BtNavigator(std::shared_ptr<::autolink::Node> node, const autonomy::tasks::proto::TaskOptions& options)
    : node_(node), options_(options) {
    AINFO << "BtNavigator creating (with node and options)";
}

BtNavigator::~BtNavigator() {}

}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy

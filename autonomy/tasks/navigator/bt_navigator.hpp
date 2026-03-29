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
 * Design aligned with nav2_bt_navigator::BtNavigator.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autolink/autolink.hpp"
#include "autolink/class_loader/class_loader_manager.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {

/**
 * @class BtNavigator
 * @brief 使用行为树进行导航的节点：加载并管理多个 Navigator 插件
 */
class BtNavigator
{
public:
    /**
     * Define BtNavigator::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(BtNavigator)

    /**
     * @brief 使用节点与 TaskOptions 构造（从 Lua 等加载的配置）
     * @param node 用于创建 action server、tf、odom 等的节点
     * @param options 任务选项（navigators、plugin_lib_names、坐标系、odom 等）
     */
    BtNavigator(std::shared_ptr<::autolink::Node> node,
                const autonomy::tasks::proto::TaskOptions& options);

    /**
     * @brief 析构
     */
    ~BtNavigator();

protected:
    // 节点（由外部设置或构造函数传入）
    std::shared_ptr<autolink::Node> node_;

    // 通过 autolink class_loader 加载并管理 Navigator 插件
    autolink::class_loader::ClassLoaderManager class_loader_manager_;
    std::vector<std::shared_ptr<common::NavigatorBase>> navigators_;
    common::NavigatorMuxer plugin_muxer_;

    // 里程计平滑，供 Navigator 反馈使用
    std::shared_ptr<control::utils::OdomSmoother> odom_smoother_;

    // 反馈与坐标系参数
    std::string robot_frame_;
    std::string global_frame_;
    double transform_tolerance_{0.1};
    double filter_duration_{0.3};
    std::string odom_topic_;

    // 变换 buffer，供 Navigator 与 BT 节点使用
    std::shared_ptr<transform::Buffer> tf_;

    // 可选配置（由带 options 的构造函数设置）
    autonomy::tasks::proto::TaskOptions options_;
};

}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy

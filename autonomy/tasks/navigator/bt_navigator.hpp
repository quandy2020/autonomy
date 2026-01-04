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
#include <mutex>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {

/**
 * @class autonomy::tasks::navigator::BtNavigator
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class BtNavigator
{
public:
    /**
     * @brief A constructor for autonomy::tasks::navigator::BtNavigator class
     * @param options Additional options to control creation of the node.
     */
    explicit BtNavigator();
    /**
     * @brief A destructor for autonomy::tasks::navigator::BtNavigator class
     */
    ~BtNavigator() = default;

protected:
    // To handle all the BT related execution
    // TODO: Re-implement with Autolink-based navigator system
    // pluginlib::ClassLoader<NavigatorBase> class_loader_;
    // std::vector<pluginlib::UniquePtr<NavigatorBase>> navigators_;
    // NavigatorMuxer plugin_muxer_;

    // Odometry smoother object - to be replaced with Autolink odometry system
    void* odom_smoother_;  // Placeholder for future Autolink odometry
                           // implementation

    // Metrics for feedback
    std::string robot_frame_;
    std::string global_frame_;
    double transform_tolerance_;
    double filter_duration_;
    std::string odom_topic_;

    // Transform buffer - to be replaced with Autolink transform system
    void*
        tf_buffer_;  // Placeholder for future Autolink transform implementation
};

}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy

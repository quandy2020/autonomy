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

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace control {
namespace common {

/**
 * @class nav2_core::ProgressChecker
 * @brief This class defines the plugin interface used to check the
 * position of the robot to make sure that it is actually progressing
 * towards a goal.
 */
class ProgressChecker
{
public:
    /**
     * Define ProgressChecker::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ProgressChecker)

    /**
     * @brief A Destructor for ProgressChecker
     */
    virtual ~ProgressChecker() = default;

    /**
     * @brief Initialize parameters for ProgressChecker
     */
    virtual void Initialize(const std::string& plugin_name) = 0;

    /**
     * @brief Checks if the robot has moved compare to previous
     * pose
     * @param current_pose Current pose of the robot
     * @return True if progress is made
     */

    virtual bool Check(commsgs::geometry_msgs::PoseStamped& current_pose) = 0;

    /**
     * @brief Reset class state upon calling
     */
    virtual void Reset() = 0;
};

}  // namespace common
}  // namespace control
}  // namespace autonomy
/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/tools/aviz/common/display.hpp"
#include "autonomy/tools/aviz/common/properties/bool_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/string_property.hpp"

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief RobotModelDisplay
 * Robot model display
 *
 * Displays robot model from URDF
 */
class RobotModelDisplay : public aviz::common::Display
{
    Q_OBJECT

public:
    explicit RobotModelDisplay(const QString& name = "RobotModelDisplay");
    ~RobotModelDisplay() override;

    void onInitialize() override;
    void onEnable() override;
    void onDisable() override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;

private Q_SLOTS:
    void updateRobotDescription();
    void updateVisualVisible();
    void updateCollisionVisible();
    void updateAlpha();

private:
    bool loadURDF();
    void updateRobot();

    aviz::common::properties::StringProperty* robot_description_property_;
    aviz::common::properties::BoolProperty* visual_enabled_property_;
    aviz::common::properties::BoolProperty* collision_enabled_property_;
    aviz::common::properties::FloatProperty* alpha_property_;

    std::string robot_description_;
    bool robot_loaded_;
    // TODO: Add URDF model and rendering objects
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

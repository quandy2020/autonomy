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

#include "autonomy/tools/aviz/plugins/displays/robot_model/robot_model_display.hpp"

#include <string>

#include "autonomy/tools/aviz/common/properties/bool_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/status_property.hpp"
#include "autonomy/tools/aviz/common/properties/string_property.hpp"

namespace aviz {
namespace plugins {
namespace displays {

RobotModelDisplay::RobotModelDisplay(const QString& name) : aviz::common::Display(), robot_loaded_(false) {
    setName(name);
    setClassId("aviz/RobotModel");

    robot_description_property_ = new aviz::common::properties::StringProperty(
        QString("Robot Description"), QString(""), QString("URDF robot description (XML string or file path)."),
        nullptr, SLOT(updateRobotDescription()), this);

    visual_enabled_property_ = new aviz::common::properties::BoolProperty(
        QString("Visual Enabled"), true, QString("Whether to show the visual representation of the robot."), nullptr,
        SLOT(updateVisualVisible()), this);

    collision_enabled_property_ = new aviz::common::properties::BoolProperty(
        QString("Collision Enabled"), false, QString("Whether to show the collision representation of the robot."),
        nullptr, SLOT(updateCollisionVisible()), this);

    alpha_property_ = new aviz::common::properties::FloatProperty(
        QString("Alpha"), 1.0f, QString("Amount of transparency to apply to the robot model."), nullptr,
        SLOT(updateAlpha()), this);
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);
}

RobotModelDisplay::~RobotModelDisplay() = default;

void RobotModelDisplay::onInitialize() {
    aviz::common::Display::onInitialize();
}

void RobotModelDisplay::onEnable() {
    aviz::common::Display::onEnable();
    updateRobot();
}

void RobotModelDisplay::onDisable() {
    aviz::common::Display::onDisable();
}

void RobotModelDisplay::update(float wall_dt, float ros_dt) {
    (void)wall_dt;
    (void)ros_dt;

    if (!isEnabled() || !robot_loaded_) {
        return;
    }

    // TODO: Update robot joint positions from TF or joint states
}

void RobotModelDisplay::reset() {
    aviz::common::Display::reset();
    robot_loaded_ = false;
    // TODO: Clear robot visualization
}

void RobotModelDisplay::updateRobotDescription() {
    robot_description_ = robot_description_property_->getStdString();
    loadURDF();
}

void RobotModelDisplay::updateVisualVisible() {
    updateRobot();
    queueRender();
}

void RobotModelDisplay::updateCollisionVisible() {
    updateRobot();
    queueRender();
}

void RobotModelDisplay::updateAlpha() {
    updateRobot();
    queueRender();
}

bool RobotModelDisplay::loadURDF() {
    if (robot_description_.empty()) {
        robot_loaded_ = false;
        return false;
    }

    // TODO: Parse URDF and create robot model
    // This would involve:
    // 1. Parse URDF XML
    // 2. Create Ogre entities for each link
    // 3. Set up joint transformations
    // 4. Apply materials and textures

    robot_loaded_ = true;
    updateRobot();
    return true;
}

void RobotModelDisplay::updateRobot() {
    if (!robot_loaded_ || !isEnabled()) {
        return;
    }

    // TODO: Update robot visualization based on properties
    // This would involve:
    // 1. Show/hide visual/collision geometry based on properties
    // 2. Update alpha/transparency
    // 3. Update joint positions from TF

    queueRender();
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

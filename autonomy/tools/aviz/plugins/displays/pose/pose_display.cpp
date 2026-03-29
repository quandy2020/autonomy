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

#include "autonomy/tools/aviz/plugins/displays/pose/pose_display.hpp"

#include <OgreSceneNode.h>

#include "autonomy/tools/aviz/common/msg_conversions.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/enum_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/rendering/objects/arrow.hpp"
#include "autonomy/tools/aviz/rendering/objects/axes.hpp"

namespace aviz {
namespace plugins {
namespace displays {

PoseDisplay::PoseDisplay()
    : AutolinkTopicDisplay<autonomy::commsgs::geometry_msgs::PoseStamped>(
          "aviz/Pose"),
      arrow_(nullptr),
      axes_(nullptr),
      pose_valid_(false) {
    shape_property_ = new aviz::common::properties::EnumProperty(
        QString("Shape"), QString("Arrow"),
        QString("Shape to display the pose as."), nullptr,
        SLOT(updateShapeChoice()), this);
    shape_property_->addOption("Arrow", Arrow);
    shape_property_->addOption("Axes", Axes);

    color_property_ = new aviz::common::properties::ColorProperty(
        QString("Color"), QColor(255, 25, 0),
        QString("Color to draw the arrow."), nullptr,
        SLOT(updateColorAndAlpha()), this);

    alpha_property_ = new aviz::common::properties::FloatProperty(
        QString("Alpha"), 1.0f,
        QString("Amount of transparency to apply to the arrow."), nullptr,
        SLOT(updateColorAndAlpha()), this);
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);

    shaft_length_property_ = new aviz::common::properties::FloatProperty(
        QString("Shaft Length"), 1.0f,
        QString("Length of the arrow's shaft, in meters."), nullptr,
        SLOT(updateArrowGeometry()), this);

    shaft_radius_property_ = new aviz::common::properties::FloatProperty(
        QString("Shaft Radius"), 0.05f,
        QString("Radius of the arrow's shaft, in meters."), nullptr,
        SLOT(updateArrowGeometry()), this);

    head_length_property_ = new aviz::common::properties::FloatProperty(
        QString("Head Length"), 0.3f,
        QString("Length of the arrow's head, in meters."), nullptr,
        SLOT(updateArrowGeometry()), this);

    head_radius_property_ = new aviz::common::properties::FloatProperty(
        QString("Head Radius"), 0.1f,
        QString("Radius of the arrow's head, in meters."), nullptr,
        SLOT(updateArrowGeometry()), this);

    axes_length_property_ = new aviz::common::properties::FloatProperty(
        QString("Axes Length"), 1.0f,
        QString("Length of each axis, in meters."), nullptr,
        SLOT(updateAxisGeometry()), this);

    axes_radius_property_ = new aviz::common::properties::FloatProperty(
        QString("Axes Radius"), 0.1f,
        QString("Radius of each axis, in meters."), nullptr,
        SLOT(updateAxisGeometry()), this);
}

PoseDisplay::~PoseDisplay() = default;

void PoseDisplay::onInitialize() {
    AutolinkTopicDisplay::onInitialize();

    if (scene_manager_ && scene_node_) {
        arrow_ = std::make_unique<aviz::rendering::Arrow>(
            scene_manager_, scene_node_, shaft_length_property_->getFloat(),
            shaft_radius_property_->getFloat(),
            head_length_property_->getFloat(),
            head_radius_property_->getFloat());
        arrow_->setDirection(Ogre::Vector3::UNIT_X);

        axes_ = std::make_unique<aviz::rendering::Axes>(
            scene_manager_, scene_node_, axes_length_property_->getFloat(),
            axes_radius_property_->getFloat());

        updateShapeChoice();
        updateColorAndAlpha();
    }
}

void PoseDisplay::onEnable() {
    AutolinkTopicDisplay::onEnable();
    updateShapeVisibility();
}

void PoseDisplay::onDisable() {
    AutolinkTopicDisplay::onDisable();
}

void PoseDisplay::reset() {
    AutolinkTopicDisplay::reset();
    pose_valid_ = false;
    if (arrow_) {
        arrow_->getSceneNode()->setVisible(false);
    }
    if (axes_) {
        axes_->getSceneNode()->setVisible(false);
    }
}

void PoseDisplay::processMessage(
    const std::shared_ptr<autonomy::commsgs::geometry_msgs::PoseStamped>& msg) {
    if (!msg || !scene_manager_ || !scene_node_) {
        return;
    }

    // For now, assume identity transform (TODO: implement frame transformation)
    Ogre::Vector3 position = aviz::common::pointMsgToOgre(msg->pose.position);
    Ogre::Quaternion orientation =
        aviz::common::quaternionMsgToOgre(msg->pose.orientation);

    auto shape = static_cast<Shape>(shape_property_->getOptionInt());

    if (shape == Arrow && arrow_) {
        arrow_->setPosition(position);
        arrow_->setOrientation(orientation);
        arrow_->getSceneNode()->setVisible(true);
        pose_valid_ = true;
    }

    if (shape == Axes && axes_) {
        axes_->setPosition(position);
        axes_->setOrientation(orientation);
        axes_->getSceneNode()->setVisible(true);
        pose_valid_ = true;
    }

    queueRender();
}

void PoseDisplay::updateShapeVisibility() {
    auto shape = static_cast<Shape>(shape_property_->getOptionInt());
    bool use_arrow = (shape == Arrow);
    bool use_axes = (shape == Axes);

    if (arrow_) {
        arrow_->getSceneNode()->setVisible(use_arrow && pose_valid_);
    }
    if (axes_) {
        axes_->getSceneNode()->setVisible(use_axes && pose_valid_);
    }

    // Show/hide properties based on shape
    head_radius_property_->setHidden(!use_arrow);
    head_length_property_->setHidden(!use_arrow);
    shaft_radius_property_->setHidden(!use_arrow);
    shaft_length_property_->setHidden(!use_arrow);
    axes_length_property_->setHidden(!use_axes);
    axes_radius_property_->setHidden(!use_axes);

    queueRender();
}

void PoseDisplay::updateColorAndAlpha() {
    QColor color = color_property_->getColor();
    float alpha = alpha_property_->getFloat();

    if (arrow_) {
        arrow_->setColor(color.redF(), color.greenF(), color.blueF(), alpha);
    }

    if (axes_) {
        axes_->setColor(color.redF(), color.greenF(), color.blueF(), alpha);
    }

    queueRender();
}

void PoseDisplay::updateShapeChoice() {
    updateShapeVisibility();
}

void PoseDisplay::updateAxisGeometry() {
    if (axes_) {
        axes_->set(axes_length_property_->getFloat(),
                   axes_radius_property_->getFloat());
        queueRender();
    }
}

void PoseDisplay::updateArrowGeometry() {
    if (arrow_) {
        arrow_->set(shaft_length_property_->getFloat(),
                    shaft_radius_property_->getFloat(),
                    head_length_property_->getFloat(),
                    head_radius_property_->getFloat());
        queueRender();
    }
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

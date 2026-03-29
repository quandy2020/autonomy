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

#include "autonomy/tools/aviz/plugins/displays/odometry/odometry_display.hpp"

#include <memory>
#include <string>

#include "autonomy/tools/aviz/common/msg_conversions.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/enum_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/int_property.hpp"
#include "autonomy/tools/aviz/rendering/objects/arrow.hpp"
#include "autonomy/tools/aviz/rendering/objects/axes.hpp"

namespace aviz {
namespace plugins {
namespace displays {

using aviz::common::pointMsgToOgre;
using aviz::common::quaternionMsgToOgre;
using aviz::common::properties::ColorProperty;
using aviz::common::properties::EnumProperty;
using aviz::common::properties::FloatProperty;
using aviz::common::properties::IntProperty;
using aviz::rendering::Arrow;
using aviz::rendering::Axes;

OdometryDisplay::OdometryDisplay()
    : AutolinkTopicDisplay<autonomy::commsgs::planning_msgs::Odometry>(
          "aviz/Odometry"),
      last_used_message_(nullptr) {
    setupProperties();
}

void OdometryDisplay::setupProperties() {
    position_tolerance_property_ = new aviz::common::properties::FloatProperty(
        QString("Position Tolerance"), 0.1f,
        QString("Distance, in meters from the last arrow dropped, that will "
                "cause a new arrow to drop."),
        nullptr, nullptr, this);
    position_tolerance_property_->setMin(0.0f);

    angle_tolerance_property_ = new aviz::common::properties::FloatProperty(
        QString("Angle Tolerance"), 0.1f,
        QString("Angular distance from the last arrow dropped, that will cause "
                "a new arrow to drop."),
        nullptr, nullptr, this);
    angle_tolerance_property_->setMin(0.0f);

    keep_property_ = new aviz::common::properties::IntProperty(
        QString("Keep"), 100,
        QString("Number of arrows to keep before removing the oldest.  0 means "
                "keep all of them."),
        nullptr, nullptr, this);
    keep_property_->setMin(0);

    shape_property_ = new aviz::common::properties::EnumProperty(
        QString("Shape"), QString("Arrow"),
        QString("Shape to display the pose as."), nullptr,
        SLOT(updateShapeChoice()), this);
    shape_property_->addOption("Arrow", ArrowShape);
    shape_property_->addOption("Axes", AxesShape);

    color_property_ = new aviz::common::properties::ColorProperty(
        QString("Color"), QColor(255, 25, 0), QString("Color of the arrows."),
        shape_property_, SLOT(updateColorAndAlpha()), this);

    alpha_property_ = new aviz::common::properties::FloatProperty(
        QString("Alpha"), 1.0f,
        QString("Amount of transparency to apply to the arrow."),
        shape_property_, SLOT(updateColorAndAlpha()), this);
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);

    shaft_length_property_ = new aviz::common::properties::FloatProperty(
        QString("Shaft Length"), 1.0f,
        QString("Length of the each arrow's shaft, in meters."),
        shape_property_, SLOT(updateArrowsGeometry()), this);

    shaft_radius_property_ = new aviz::common::properties::FloatProperty(
        QString("Shaft Radius"), 0.05f,
        QString("Radius of the each arrow's shaft, in meters."),
        shape_property_, SLOT(updateArrowsGeometry()), this);

    head_length_property_ = new aviz::common::properties::FloatProperty(
        QString("Head Length"), 0.3f,
        QString("Length of the each arrow's head, in meters."), shape_property_,
        SLOT(updateArrowsGeometry()), this);

    head_radius_property_ = new aviz::common::properties::FloatProperty(
        QString("Head Radius"), 0.1f,
        QString("Radius of the each arrow's head, in meters."), shape_property_,
        SLOT(updateArrowsGeometry()), this);

    axes_length_property_ = new aviz::common::properties::FloatProperty(
        QString("Axes Length"), 1.0f,
        QString("Length of each axis, in meters."), shape_property_,
        SLOT(updateAxisGeometry()), this);

    axes_radius_property_ = new aviz::common::properties::FloatProperty(
        QString("Axes Radius"), 0.1f,
        QString("Radius of each axis, in meters."), shape_property_,
        SLOT(updateAxisGeometry()), this);
}

OdometryDisplay::~OdometryDisplay() = default;

void OdometryDisplay::onInitialize() {
    AutolinkTopicDisplay::onInitialize();
    updateShapeChoice();
}

void OdometryDisplay::onEnable() {
    AutolinkTopicDisplay::onEnable();
    updateShapeVisibility();
}

void OdometryDisplay::reset() {
    AutolinkTopicDisplay::reset();
    clear();
}

void OdometryDisplay::update(float wall_dt, float ros_dt) {
    Q_UNUSED(wall_dt);
    Q_UNUSED(ros_dt);
    // No per-frame update needed
}

void OdometryDisplay::processMessage(
    const std::shared_ptr<autonomy::commsgs::planning_msgs::Odometry>& msg) {
    if (!msg || !messageIsValid(msg)) {
        return;
    }

    if (messageIsSimilarToPrevious(msg)) {
        return;
    }

    // For now, assume identity transform (TODO: implement frame transformation)
    Ogre::Vector3 position = pointMsgToOgre(msg->pose.pose.position);
    Ogre::Quaternion orientation =
        quaternionMsgToOgre(msg->pose.pose.orientation);

    auto shape = static_cast<Shape>(shape_property_->getOptionInt());
    bool use_arrow = (shape == ArrowShape);
    bool use_axes = (shape == AxesShape);

    if (use_arrow) {
        auto arrow = createAndSetArrow(position, orientation, true);
        if (arrow) {
            arrows_.push_back(std::move(arrow));
        }
    }

    if (use_axes) {
        auto axes = createAndSetAxes(position, orientation, true);
        if (axes) {
            axes_.push_back(std::move(axes));
        }
    }

    // Keep only the last N items
    int keep = keep_property_->getInt();
    if (keep > 0) {
        while (static_cast<int>(arrows_.size()) > keep) {
            arrows_.pop_front();
        }
        while (static_cast<int>(axes_.size()) > keep) {
            axes_.pop_front();
        }
    }

    last_used_message_ = msg;
    queueRender();
}

void OdometryDisplay::updateCovariances() {
    // TODO: Implement covariance visualization
    queueRender();
}

void OdometryDisplay::updateShapeChoice() {
    updateShapeVisibility();
    clear();
}

void OdometryDisplay::updateShapeVisibility() {
    auto shape = static_cast<Shape>(shape_property_->getOptionInt());
    bool use_arrow = (shape == ArrowShape);
    bool use_axes = (shape == AxesShape);

    // Show/hide properties based on shape
    color_property_->setHidden(!use_arrow && !use_axes);
    alpha_property_->setHidden(!use_arrow && !use_axes);
    head_radius_property_->setHidden(!use_arrow);
    head_length_property_->setHidden(!use_arrow);
    shaft_radius_property_->setHidden(!use_arrow);
    shaft_length_property_->setHidden(!use_arrow);
    axes_length_property_->setHidden(!use_axes);
    axes_radius_property_->setHidden(!use_axes);
}

void OdometryDisplay::updateColorAndAlpha() {
    QColor color = color_property_->getColor();
    float alpha = alpha_property_->getFloat();

    for (auto& arrow : arrows_) {
        if (arrow) {
            arrow->setColor(color.redF(), color.greenF(), color.blueF(), alpha);
        }
    }

    for (auto& axes : axes_) {
        if (axes) {
            axes->setColor(color.redF(), color.greenF(), color.blueF(), alpha);
        }
    }

    queueRender();
}

void OdometryDisplay::updateArrowsGeometry() {
    for (auto& arrow : arrows_) {
        if (arrow) {
            arrow->set(shaft_length_property_->getFloat(),
                       shaft_radius_property_->getFloat(),
                       head_length_property_->getFloat(),
                       head_radius_property_->getFloat());
        }
    }
    queueRender();
}

void OdometryDisplay::updateAxisGeometry() {
    for (auto& axes : axes_) {
        if (axes) {
            axes->set(axes_length_property_->getFloat(),
                      axes_radius_property_->getFloat());
        }
    }
    queueRender();
}

void OdometryDisplay::updateArrow(const std::unique_ptr<Arrow>& arrow) {
    if (!arrow) {
        return;
    }
    QColor color = color_property_->getColor();
    arrow->setColor(color.redF(), color.greenF(), color.blueF(),
                    alpha_property_->getFloat());
    arrow->set(
        shaft_length_property_->getFloat(), shaft_radius_property_->getFloat(),
        head_length_property_->getFloat(), head_radius_property_->getFloat());
}

void OdometryDisplay::updateAxes(const std::unique_ptr<Axes>& axes) {
    if (!axes) {
        return;
    }
    QColor color = color_property_->getColor();
    axes->setColor(color.redF(), color.greenF(), color.blueF(),
                   alpha_property_->getFloat());
    axes->set(axes_length_property_->getFloat(),
              axes_radius_property_->getFloat());
}

bool OdometryDisplay::messageIsValid(
    const std::shared_ptr<autonomy::commsgs::planning_msgs::Odometry>&
        message) {
    // Basic validation
    return message != nullptr;
}

bool OdometryDisplay::messageIsSimilarToPrevious(
    const std::shared_ptr<autonomy::commsgs::planning_msgs::Odometry>&
        message) {
    if (!last_used_message_) {
        return false;
    }

    Ogre::Vector3 last_pos =
        pointMsgToOgre(last_used_message_->pose.pose.position);
    Ogre::Vector3 current_pos = pointMsgToOgre(message->pose.pose.position);
    float position_diff = (current_pos - last_pos).length();

    if (position_diff > position_tolerance_property_->getFloat()) {
        return false;
    }

    Ogre::Quaternion last_orient =
        quaternionMsgToOgre(last_used_message_->pose.pose.orientation);
    Ogre::Quaternion current_orient =
        quaternionMsgToOgre(message->pose.pose.orientation);
    // Calculate angle difference between quaternions
    Ogre::Quaternion diff = last_orient.Inverse() * current_orient;
    float angle_diff = 2.0f * std::acos(std::abs(diff.w));

    if (angle_diff > angle_tolerance_property_->getFloat()) {
        return false;
    }

    return true;
}

std::unique_ptr<Arrow> OdometryDisplay::createAndSetArrow(
    const Ogre::Vector3& position, const Ogre::Quaternion& orientation,
    bool use_arrow) {
    if (!use_arrow || !scene_manager_ || !scene_node_) {
        return nullptr;
    }

    auto arrow = std::make_unique<Arrow>(
        scene_manager_, scene_node_, shaft_length_property_->getFloat(),
        shaft_radius_property_->getFloat(), head_length_property_->getFloat(),
        head_radius_property_->getFloat());

    QColor color = color_property_->getColor();
    arrow->setColor(color.redF(), color.greenF(), color.blueF(),
                    alpha_property_->getFloat());
    arrow->setPosition(position);
    arrow->setOrientation(orientation);

    return arrow;
}

std::unique_ptr<Axes> OdometryDisplay::createAndSetAxes(
    const Ogre::Vector3& position, const Ogre::Quaternion& orientation,
    bool use_axes) {
    if (!use_axes || !scene_manager_ || !scene_node_) {
        return nullptr;
    }

    auto axes = std::make_unique<Axes>(scene_manager_, scene_node_,
                                       axes_length_property_->getFloat(),
                                       axes_radius_property_->getFloat());

    QColor color = color_property_->getColor();
    axes->setColor(color.redF(), color.greenF(), color.blueF(),
                   alpha_property_->getFloat());
    axes->setPosition(position);
    axes->setOrientation(orientation);

    return axes;
}

void OdometryDisplay::clear() {
    arrows_.clear();
    axes_.clear();
    last_used_message_.reset();
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

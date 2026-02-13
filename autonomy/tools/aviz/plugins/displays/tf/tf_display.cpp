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

#include "autonomy/tools/aviz/plugins/displays/tf/tf_display.hpp"

#include <map>
#include <string>

#include <OgreSceneNode.h>

#include "autonomy/tools/aviz/common/properties/bool_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/rendering/objects/arrow.hpp"
#include "autonomy/tools/aviz/rendering/objects/axes.hpp"

namespace aviz {
namespace plugins {
namespace displays {

TfDisplay::TfDisplay(const QString& name) : aviz::common::Display(), last_update_time_(0.0f) {
    setName(name);
    setClassId("aviz/TF");

    show_names_property_ = new aviz::common::properties::BoolProperty(
        QString("Show Names"), true, QString("Whether to show frame names."), nullptr, SLOT(updateShowNames()), this);

    show_axes_property_ = new aviz::common::properties::BoolProperty(
        QString("Show Axes"), true, QString("Whether to show coordinate axes for each frame."), nullptr,
        SLOT(updateShowAxes()), this);

    show_arrows_property_ = new aviz::common::properties::BoolProperty(
        QString("Show Arrows"), false, QString("Whether to show arrows connecting parent and child frames."), nullptr,
        SLOT(updateShowArrows()), this);

    frame_timeout_property_ = new aviz::common::properties::FloatProperty(
        QString("Frame Timeout"), 15.0f,
        QString("How long to keep showing a frame that has stopped being published [s]."), nullptr,
        SLOT(updateFrameEnabled()), this);
    frame_timeout_property_->setMin(0.0f);

    update_interval_property_ = new aviz::common::properties::FloatProperty(
        QString("Update Interval"), 0.0f,
        QString("The interval, in seconds, at which to update the frame list. 0 means update every frame."), nullptr,
        SLOT(updateFrameEnabled()), this);
    update_interval_property_->setMin(0.0f);

    axes_length_property_ = new aviz::common::properties::FloatProperty(QString("Axes Length"), 0.1f,
                                                                        QString("Length of each axis, in meters."),
                                                                        nullptr, SLOT(updateShowAxes()), this);
    axes_length_property_->setMin(0.01f);

    axes_radius_property_ = new aviz::common::properties::FloatProperty(QString("Axes Radius"), 0.01f,
                                                                        QString("Radius of each axis, in meters."),
                                                                        nullptr, SLOT(updateShowAxes()), this);
    axes_radius_property_->setMin(0.001f);

    arrow_length_property_ = new aviz::common::properties::FloatProperty(QString("Arrow Length"), 0.3f,
                                                                         QString("Length of each arrow, in meters."),
                                                                         nullptr, SLOT(updateShowArrows()), this);
    arrow_length_property_->setMin(0.01f);

    arrow_radius_property_ = new aviz::common::properties::FloatProperty(QString("Arrow Radius"), 0.02f,
                                                                         QString("Radius of each arrow, in meters."),
                                                                         nullptr, SLOT(updateShowArrows()), this);
    arrow_radius_property_->setMin(0.001f);
}

TfDisplay::~TfDisplay() = default;

void TfDisplay::onInitialize() {
    aviz::common::Display::onInitialize();
}

void TfDisplay::onEnable() {
    aviz::common::Display::onEnable();
    updateTransforms();
}

void TfDisplay::onDisable() {
    aviz::common::Display::onDisable();
    // Hide all frame visuals
    for (auto& [frame_id, axes] : frame_axes_) {
        if (axes) {
            axes->getSceneNode()->setVisible(false);
        }
    }
    for (auto& [frame_id, arrow] : frame_arrows_) {
        if (arrow) {
            arrow->getSceneNode()->setVisible(false);
        }
    }
}

void TfDisplay::update(float wall_dt, float ros_dt) {
    (void)wall_dt;
    (void)ros_dt;

    if (!isEnabled() || !context_) {
        return;
    }

    // Update transforms periodically
    float update_interval = update_interval_property_->getFloat();
    last_update_time_ += wall_dt;
    if (update_interval > 0.0f && last_update_time_ < update_interval) {
        return;
    }
    last_update_time_ = 0.0f;

    updateTransforms();
}

void TfDisplay::reset() {
    aviz::common::Display::reset();
    frame_axes_.clear();
    frame_arrows_.clear();
    last_update_time_ = 0.0f;
}

void TfDisplay::updateTransforms() {
    // TODO: Get transforms from FrameManager
    // For now, this is a placeholder that would query the TF tree
    // and create/update frame visuals

    // Example: Get all frames from FrameManager
    // std::vector<std::string> frames = context_->getFrameManager()->getAllFrameNames();
    // for (const auto& frame_id : frames) {
    //     std::string parent_frame;
    //     Ogre::Vector3 position;
    //     Ogre::Quaternion orientation;
    //     if (context_->getFrameManager()->getTransform(frame_id, parent_frame, position, orientation)) {
    //         createFrameVisual(frame_id, parent_frame, position, orientation);
    //     }
    // }
}

void TfDisplay::createFrameVisual(const std::string& frame_id, const std::string& parent_frame_id,
                                  const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {
    if (show_axes_property_->getBool()) {
        if (frame_axes_.find(frame_id) == frame_axes_.end()) {
            frame_axes_[frame_id] = std::make_unique<aviz::rendering::Axes>(
                scene_manager_, scene_node_, axes_length_property_->getFloat(), axes_radius_property_->getFloat());
        }
        auto& axes = frame_axes_[frame_id];
        axes->setPosition(position);
        axes->setOrientation(orientation);
        axes->getSceneNode()->setVisible(true);
    }

    if (show_arrows_property_->getBool() && !parent_frame_id.empty()) {
        std::string arrow_key = parent_frame_id + "->" + frame_id;
        if (frame_arrows_.find(arrow_key) == frame_arrows_.end()) {
            frame_arrows_[arrow_key] = std::make_unique<aviz::rendering::Arrow>(
                scene_manager_, scene_node_, arrow_length_property_->getFloat(), arrow_radius_property_->getFloat(),
                arrow_length_property_->getFloat() * 0.3f, arrow_radius_property_->getFloat() * 1.5f);
        }
        auto& arrow = frame_arrows_[arrow_key];
        arrow->setPosition(position);
        // TODO: Set arrow direction based on parent->child relationship
        arrow->getSceneNode()->setVisible(true);
    }
}

void TfDisplay::updateShowNames() {
    // TODO: Implement text rendering for frame names
    queueRender();
}

void TfDisplay::updateShowAxes() {
    bool show = show_axes_property_->getBool();
    for (auto& [frame_id, axes] : frame_axes_) {
        if (axes) {
            axes->getSceneNode()->setVisible(show && isEnabled());
            if (show) {
                axes->set(axes_length_property_->getFloat(), axes_radius_property_->getFloat());
            }
        }
    }
    queueRender();
}

void TfDisplay::updateShowArrows() {
    bool show = show_arrows_property_->getBool();
    for (auto& [arrow_key, arrow] : frame_arrows_) {
        if (arrow) {
            arrow->getSceneNode()->setVisible(show && isEnabled());
        }
    }
    queueRender();
}

void TfDisplay::updateFrameEnabled() {
    updateTransforms();
    queueRender();
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

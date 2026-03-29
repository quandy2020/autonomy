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

#include "autonomy/tools/aviz/plugins/displays/marker/marker_display.hpp"

#include <OgreSceneNode.h>

#include <sstream>

#include "autonomy/tools/aviz/common/msg_conversions.hpp"
#include "autonomy/tools/aviz/common/properties/status_property.hpp"
#include "autonomy/tools/aviz/rendering/objects/arrow.hpp"
#include "autonomy/tools/aviz/rendering/objects/axes.hpp"
#include "autonomy/tools/aviz/rendering/objects/billboard_line.hpp"
#include "autonomy/tools/aviz/rendering/objects/shape.hpp"

namespace aviz {
namespace plugins {
namespace displays {

MarkerDisplay::MarkerDisplay(const QString& name)
    : AutolinkTopicDisplay<autonomy::commsgs::visualization_msgs::Marker>(
          QString("aviz/Marker")) {
    setName(name);
}

MarkerDisplay::~MarkerDisplay() = default;

void MarkerDisplay::onInitialize() {
    AutolinkTopicDisplay::onInitialize();
}

void MarkerDisplay::reset() {
    AutolinkTopicDisplay::reset();
    deleteAllMarkers();
}

void MarkerDisplay::processMarkerMessage(
    const std::shared_ptr<autonomy::commsgs::visualization_msgs::Marker>& msg) {
    processMessage(msg);
}

void MarkerDisplay::processMessage(
    const std::shared_ptr<autonomy::commsgs::visualization_msgs::Marker>& msg) {
    if (!msg || !scene_manager_ || !scene_node_) {
        return;
    }

    // For now, assume identity transform (TODO: implement frame transformation)
    setTransformOk();

    std::string key = msg->ns + "/" + std::to_string(msg->id);

    // Handle marker actions
    if (msg->action == 2) {  // DELETE
        deleteMarker(msg->ns, msg->id);
    } else if (msg->action == 3) {  // DELETEALL
        deleteAllMarkers(msg->ns);
    } else {  // ADD or MODIFY
        createMarker(*msg);
    }

    queueRender();
}

void MarkerDisplay::deleteMarker(const std::string& ns, int32_t id) {
    std::string key = ns + "/" + std::to_string(id);
    shape_markers_.erase(key);
    arrow_markers_.erase(key);
    axes_markers_.erase(key);
    line_markers_.erase(key);
}

void MarkerDisplay::deleteAllMarkers(const std::string& ns) {
    if (ns.empty()) {
        shape_markers_.clear();
        arrow_markers_.clear();
        axes_markers_.clear();
        line_markers_.clear();
    } else {
        // Delete markers with matching namespace
        auto it = shape_markers_.begin();
        while (it != shape_markers_.end()) {
            if (it->first.find(ns + "/") == 0) {
                it = shape_markers_.erase(it);
            } else {
                ++it;
            }
        }
        // Similar for other marker types...
    }
}

void MarkerDisplay::createMarker(
    const autonomy::commsgs::visualization_msgs::Marker& marker) {
    std::string key = marker.ns + "/" + std::to_string(marker.id);

    // Delete existing marker with same key
    deleteMarker(marker.ns, marker.id);

    Ogre::Vector3 position = aviz::common::pointMsgToOgre(marker.pose.position);
    Ogre::Quaternion orientation =
        aviz::common::quaternionMsgToOgre(marker.pose.orientation);
    Ogre::Vector3 scale_vec(static_cast<float>(marker.scale.x),
                            static_cast<float>(marker.scale.y),
                            static_cast<float>(marker.scale.z));

    Ogre::ColourValue color(
        static_cast<float>(marker.color.r), static_cast<float>(marker.color.g),
        static_cast<float>(marker.color.b), static_cast<float>(marker.color.a));

    // Create marker based on type
    switch (marker.type) {
        case 0: {  // ARROW
            auto arrow = std::make_unique<aviz::rendering::Arrow>(
                scene_manager_, scene_node_);
            arrow->setPosition(position);
            arrow->setOrientation(orientation);
            arrow->set(scale_vec.x, scale_vec.y * 0.1f, scale_vec.z,
                       scale_vec.z);
            arrow->setColor(color.r, color.g, color.b, color.a);
            arrow_markers_[key] = std::move(arrow);
            break;
        }
        case 1: {  // CUBE
            auto shape = std::make_unique<aviz::rendering::Shape>(
                aviz::rendering::Shape::Cube, scene_manager_, scene_node_);
            shape->setPosition(position);
            shape->setOrientation(orientation);
            shape->setScale(scale_vec);
            shape->setColor(color.r, color.g, color.b, color.a);
            shape_markers_[key] = std::move(shape);
            break;
        }
        case 2: {  // SPHERE
            auto shape = std::make_unique<aviz::rendering::Shape>(
                aviz::rendering::Shape::Sphere, scene_manager_, scene_node_);
            shape->setPosition(position);
            shape->setOrientation(orientation);
            shape->setScale(scale_vec);
            shape->setColor(color.r, color.g, color.b, color.a);
            shape_markers_[key] = std::move(shape);
            break;
        }
        case 3: {  // CYLINDER
            auto shape = std::make_unique<aviz::rendering::Shape>(
                aviz::rendering::Shape::Cylinder, scene_manager_, scene_node_);
            shape->setPosition(position);
            shape->setOrientation(orientation);
            shape->setScale(scale_vec);
            shape->setColor(color.r, color.g, color.b, color.a);
            shape_markers_[key] = std::move(shape);
            break;
        }
        // TODO: Implement other marker types (LINE_STRIP, LINE_LIST, POINTS,
        // etc.)
        default:
            // Unsupported marker type
            break;
    }
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

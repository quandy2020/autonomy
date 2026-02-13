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

#include "autonomy/tools/aviz/plugins/displays/map/map_display.hpp"

#include <OgreSceneNode.h>

#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/status_property.hpp"

namespace aviz {
namespace plugins {
namespace displays {

MapDisplay::MapDisplay(const QString& name)
    : AutolinkTopicDisplay<autonomy::commsgs::map_msgs::OccupancyGrid>(QString("aviz/Map")) {
    setName(name);
    alpha_property_ = new aviz::common::properties::FloatProperty(
        QString("Alpha"), 0.7f, QString("Amount of transparency to apply to the map."), nullptr, SLOT(updateAlpha()),
        this);
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);

    color_scheme_property_ = new aviz::common::properties::ColorProperty(QString("Color Scheme"), QColor(0, 0, 0),
                                                                         QString("Color scheme for the map."), nullptr,
                                                                         SLOT(updateAlpha()), this);
}

MapDisplay::~MapDisplay() = default;

void MapDisplay::onInitialize() {
    AutolinkTopicDisplay::onInitialize();
}

void MapDisplay::reset() {
    AutolinkTopicDisplay::reset();
    map_vertices_.clear();
    map_colors_.clear();
}

void MapDisplay::processMessage(const std::shared_ptr<autonomy::commsgs::map_msgs::OccupancyGrid>& msg) {
    if (!msg || !scene_manager_ || !scene_node_) {
        return;
    }

    // For now, assume identity transform (TODO: implement frame transformation)
    setTransformOk();

    // TODO: Convert OccupancyGrid to Ogre mesh/quads for rendering
    // This would involve:
    // 1. Iterate through map data
    // 2. Create vertices for occupied cells
    // 3. Apply color based on occupancy value
    // 4. Render as quads or triangles

    queueRender();
}

void MapDisplay::updateAlpha() {
    queueRender();
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

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

#include "autonomy/commsgs/map_msgs.hpp"

#include "autonomy/tools/aviz/common/autolink_topic_display.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief MapDisplay
 * Map display
 *
 * Displays map_msgs::OccupancyGrid messages
 */
class MapDisplay : public AutolinkTopicDisplay<autonomy::commsgs::map_msgs::OccupancyGrid>
{
    Q_OBJECT

public:
    explicit MapDisplay(const QString& name = "MapDisplay");
    ~MapDisplay() override;

    void onInitialize() override;
    void reset() override;

protected:
    void processMessage(const std::shared_ptr<autonomy::commsgs::map_msgs::OccupancyGrid>& msg) override;

private Q_SLOTS:
    void updateAlpha();

private:
    aviz::common::properties::FloatProperty* alpha_property_;
    aviz::common::properties::ColorProperty* color_scheme_property_;

    // Map visualization data
    std::vector<Ogre::Vector3> map_vertices_;
    std::vector<Ogre::ColourValue> map_colors_;
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

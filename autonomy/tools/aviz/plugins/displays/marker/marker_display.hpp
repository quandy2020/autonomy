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

#include <map>
#include <memory>
#include <string>

#include "autonomy/commsgs/visualization_msgs.hpp"

#include "autonomy/tools/aviz/common/autolink_topic_display.hpp"

namespace aviz {
namespace rendering {
class Shape;
class Arrow;
class Axes;
class BillboardLine;
}  // namespace rendering
}  // namespace aviz

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief MarkerDisplay
 * Similar to rviz_default_plugins::displays::MarkerDisplay
 *
 * Displays visualization_msgs::Marker messages
 */
class MarkerDisplay : public AutolinkTopicDisplay<autonomy::commsgs::visualization_msgs::Marker>
{
    Q_OBJECT

public:
    explicit MarkerDisplay(const QString& name = "MarkerDisplay");
    ~MarkerDisplay() override;

    void onInitialize() override;
    void reset() override;

    // Public method to process a marker message (used by MarkerArrayDisplay)
    void processMarkerMessage(const std::shared_ptr<autonomy::commsgs::visualization_msgs::Marker>& msg);

protected:
    void processMessage(const std::shared_ptr<autonomy::commsgs::visualization_msgs::Marker>& msg) override;

private:
    void deleteMarker(const std::string& ns, int32_t id);
    void deleteAllMarkers(const std::string& ns = "");
    void createMarker(const autonomy::commsgs::visualization_msgs::Marker& marker);

    // Marker storage: key is "ns/id"
    std::map<std::string, std::unique_ptr<aviz::rendering::Shape>> shape_markers_;
    std::map<std::string, std::unique_ptr<aviz::rendering::Arrow>> arrow_markers_;
    std::map<std::string, std::unique_ptr<aviz::rendering::Axes>> axes_markers_;
    std::map<std::string, std::unique_ptr<aviz::rendering::BillboardLine>> line_markers_;
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

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

#include "autonomy/commsgs/geometry_msgs.hpp"

#include "autonomy/tools/aviz/common/autolink_topic_display.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/enum_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"

namespace aviz {
namespace rendering {
class Arrow;
class Axes;
}  // namespace rendering
}  // namespace aviz

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief PoseDisplay
 * Similar to rviz_default_plugins::displays::PoseDisplay
 *
 * Accumulates and displays the pose from a geometry_msgs::PoseStamped message
 */
class PoseDisplay : public AutolinkTopicDisplay<autonomy::commsgs::geometry_msgs::PoseStamped>
{
    Q_OBJECT

public:
    enum Shape {
        Arrow,
        Axes,
    };

    PoseDisplay();
    ~PoseDisplay() override;

    void onInitialize() override;

    void reset() override;

protected:
    /** @brief Overridden from AutolinkTopicDisplay to get arrow/axes visibility correct. */
    void onEnable() override;
    void onDisable() override;

    void processMessage(const std::shared_ptr<autonomy::commsgs::geometry_msgs::PoseStamped>& msg) override;

private Q_SLOTS:
    void updateShapeVisibility();
    void updateColorAndAlpha();
    void updateShapeChoice();
    void updateAxisGeometry();
    void updateArrowGeometry();

private:
    std::unique_ptr<aviz::rendering::Arrow> arrow_;
    std::unique_ptr<aviz::rendering::Axes> axes_;
    bool pose_valid_;

    aviz::common::properties::EnumProperty* shape_property_;

    aviz::common::properties::ColorProperty* color_property_;
    aviz::common::properties::FloatProperty* alpha_property_;

    aviz::common::properties::FloatProperty* head_radius_property_;
    aviz::common::properties::FloatProperty* head_length_property_;
    aviz::common::properties::FloatProperty* shaft_radius_property_;
    aviz::common::properties::FloatProperty* shaft_length_property_;

    aviz::common::properties::FloatProperty* axes_length_property_;
    aviz::common::properties::FloatProperty* axes_radius_property_;
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

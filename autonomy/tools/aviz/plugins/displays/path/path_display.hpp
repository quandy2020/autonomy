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

#include <vector>

#include "autonomy/commsgs/planning_msgs.hpp"

#include "autonomy/tools/aviz/common/autolink_topic_display.hpp"
#include "autonomy/tools/aviz/common/properties/color_property.hpp"
#include "autonomy/tools/aviz/common/properties/enum_property.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"
#include "autonomy/tools/aviz/common/properties/int_property.hpp"
#include "autonomy/tools/aviz/common/properties/vector_property.hpp"

namespace Ogre {
class ManualObject;
}

namespace aviz {
namespace rendering {
class BillboardLine;
class Arrow;
class Axes;
}  // namespace rendering
}  // namespace aviz

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief PathDisplay
 * Path display
 *
 * Displays a planning_msgs::Path message as a line connecting poses
 */
class PathDisplay : public AutolinkTopicDisplay<autonomy::commsgs::planning_msgs::Path>
{
    Q_OBJECT

public:
    PathDisplay();
    ~PathDisplay() override;

    /** @brief Overridden from Display. */
    void reset() override;

    /** @brief Overridden from AutolinkTopicDisplay. */
    void processMessage(const std::shared_ptr<autonomy::commsgs::planning_msgs::Path>& msg) override;

protected:
    /** @brief Overridden from Display. */
    void onInitialize() override;

private Q_SLOTS:
    void updateBufferLength();
    void updateStyle();
    void updateLineWidth();
    void updateOffset();
    void updatePoseStyle();
    void updatePoseAxisGeometry();
    void updatePoseArrowColor();
    void updatePoseArrowGeometry();

private:
    void destroyObjects();
    void allocateArrowVector(std::vector<aviz::rendering::Arrow*>& arrow_vect, size_t num);
    void allocateAxesVector(std::vector<aviz::rendering::Axes*>& axes_vect, size_t num);
    void destroyPoseAxesChain();
    void destroyPoseArrowChain();
    void updateManualObject(Ogre::ManualObject* manual_object, const autonomy::commsgs::planning_msgs::Path& msg,
                            const Ogre::Matrix4& transform);
    void updateBillBoardLine(aviz::rendering::BillboardLine* billboard_line,
                             const autonomy::commsgs::planning_msgs::Path& msg, const Ogre::Matrix4& transform);
    void updatePoseMarkers(size_t buffer_index, const autonomy::commsgs::planning_msgs::Path& msg,
                           const Ogre::Matrix4& transform);
    void updateAxesMarkers(std::vector<aviz::rendering::Axes*>& axes_vect,
                           const autonomy::commsgs::planning_msgs::Path& msg, const Ogre::Matrix4& transform);
    void updateArrowMarkers(std::vector<aviz::rendering::Arrow*>& arrow_vect,
                            const autonomy::commsgs::planning_msgs::Path& msg, const Ogre::Matrix4& transform);

    std::vector<Ogre::ManualObject*> manual_objects_;
    std::vector<aviz::rendering::BillboardLine*> billboard_lines_;
    std::vector<std::vector<aviz::rendering::Axes*>> axes_chain_;
    std::vector<std::vector<aviz::rendering::Arrow*>> arrow_chain_;
    Ogre::MaterialPtr lines_material_;

    aviz::common::properties::EnumProperty* style_property_;
    aviz::common::properties::ColorProperty* color_property_;
    aviz::common::properties::FloatProperty* alpha_property_;
    aviz::common::properties::FloatProperty* line_width_property_;
    aviz::common::properties::IntProperty* buffer_length_property_;
    aviz::common::properties::VectorProperty* offset_property_;

    enum LineStyle { LINES, BILLBOARDS };

    // pose marker property
    aviz::common::properties::EnumProperty* pose_style_property_;
    aviz::common::properties::FloatProperty* pose_axes_length_property_;
    aviz::common::properties::FloatProperty* pose_axes_radius_property_;
    aviz::common::properties::ColorProperty* pose_arrow_color_property_;
    aviz::common::properties::FloatProperty* pose_arrow_shaft_length_property_;
    aviz::common::properties::FloatProperty* pose_arrow_head_length_property_;
    aviz::common::properties::FloatProperty* pose_arrow_shaft_diameter_property_;
    aviz::common::properties::FloatProperty* pose_arrow_head_diameter_property_;

    enum PoseStyle {
        NONE,
        AXES,
        ARROWS,
    };
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

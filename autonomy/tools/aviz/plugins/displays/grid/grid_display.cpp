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

#include "autonomy/tools/aviz/plugins/displays/grid/grid_display.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <cstdint>
#include <memory>
#include <string>

#include "autonomy/tools/aviz/common/display_context.hpp"
#include "autonomy/tools/aviz/common/properties/parse_color.hpp"
#include "autonomy/tools/aviz/common/properties/property.hpp"
#include "autonomy/tools/aviz/rendering/objects/grid.hpp"

namespace aviz {
namespace plugins {
namespace displays {

using aviz::common::properties::ColorProperty;
using aviz::common::properties::EnumProperty;
using aviz::common::properties::FloatProperty;
using aviz::common::properties::IntProperty;
using aviz::common::properties::qtToOgre;
using aviz::common::properties::VectorProperty;
using aviz::rendering::Grid;

GridDisplay::GridDisplay() {
    cell_count_property_ = new IntProperty(
        QString("Plane Cell Count"), 10,
        QString("The number of cells to draw in the plane of the grid."),
        nullptr, SLOT(updateCellCount()), this);
    cell_count_property_->setMin(1);

    height_property_ = new IntProperty(
        QString("Normal Cell Count"), 0,
        QString(
            "The number of cells to draw along the normal vector of the grid. "
            " Setting to anything but 0 makes the grid 3D."),
        nullptr, SLOT(updateHeight()), this);
    height_property_->setMin(0);

    cell_size_property_ = new FloatProperty(
        QString("Cell Size"), 1.0f,
        QString("The length, in meters, of the side of each cell."), nullptr,
        SLOT(updateCellSize()), this);
    cell_size_property_->setMin(0.0001f);

    style_property_ = new EnumProperty(
        QString("Line Style"), QString("Lines"),
        QString("The rendering operation to use to draw the grid lines."),
        nullptr, SLOT(updateStyle()), this);
    style_property_->addOption("Lines", Grid::Lines);
    style_property_->addOption("Billboards", Grid::Billboards);

    line_width_property_ =
        new FloatProperty(QString("Line Width"), 0.03f,
                          QString("The width, in meters, of each grid line."),
                          style_property_, SLOT(updateLineWidth()), this);
    line_width_property_->setMin(0.001f);
    line_width_property_->hide();

    color_property_ = new ColorProperty(QString("Color"), Qt::gray,
                                        QString("The color of the grid lines."),
                                        nullptr, SLOT(updateColor()), this);
    alpha_property_ = new FloatProperty(
        QString("Alpha"), 0.5f,
        QString("The amount of transparency to apply to the grid lines."),
        nullptr, SLOT(updateColor()), this);
    alpha_property_->setMin(0.0f);
    alpha_property_->setMax(1.0f);

    plane_property_ =
        new EnumProperty(QString("Plane"), QString("XY"),
                         QString("The plane to draw the grid along."), nullptr,
                         SLOT(updatePlane()), this);
    plane_property_->addOption("XY", XY);
    plane_property_->addOption("XZ", XZ);
    plane_property_->addOption("YZ", YZ);

    offset_property_ = new VectorProperty(
        QString("Offset"), Ogre::Vector3::ZERO,
        QString("Allows you to offset the grid from the origin of the "
                "reference frame.  In meters."),
        nullptr, SLOT(updateOffset()), this);
}

GridDisplay::~GridDisplay() = default;

void GridDisplay::onInitialize() {
    QColor color = color_property_->getColor();
    color.setAlphaF(alpha_property_->getFloat());

    if (scene_manager_ && scene_node_) {
        grid_ = std::make_unique<Grid>(
            scene_manager_, scene_node_,
            (Grid::Style)style_property_->getOptionInt(),
            cell_count_property_->getInt(), cell_size_property_->getFloat(),
            line_width_property_->getFloat(), qtToOgre(color));

        grid_->getSceneNode()->setVisible(false);
        updatePlane();
    }
}

void GridDisplay::update(float wall_dt, float ros_dt) {
    Q_UNUSED(wall_dt);
    Q_UNUSED(ros_dt);
    // Grid is static, just make it visible if enabled
    if (grid_ && isEnabled()) {
        grid_->getSceneNode()->setVisible(true);
    } else if (grid_) {
        grid_->getSceneNode()->setVisible(false);
    }
}

void GridDisplay::updateColor() {
    if (!grid_) {
        return;
    }
    QColor color = color_property_->getColor();
    color.setAlphaF(alpha_property_->getFloat());
    grid_->setColor(qtToOgre(color));
    queueRender();
}

void GridDisplay::updateCellSize() {
    if (grid_) {
        grid_->setCellLength(cell_size_property_->getFloat());
        queueRender();
    }
}

void GridDisplay::updateCellCount() {
    if (grid_) {
        grid_->setCellCount(cell_count_property_->getInt());
        queueRender();
    }
}

void GridDisplay::updateLineWidth() {
    if (grid_) {
        grid_->setLineWidth(line_width_property_->getFloat());
        queueRender();
    }
}

void GridDisplay::updateHeight() {
    if (grid_) {
        grid_->setHeight(height_property_->getInt());
        queueRender();
    }
}

void GridDisplay::updateStyle() {
    if (!grid_) {
        return;
    }
    auto style = static_cast<Grid::Style>(style_property_->getOptionInt());
    grid_->setStyle(style);

    switch (style) {
        case Grid::Billboards:
            line_width_property_->show();
            break;
        case Grid::Lines:
        default:
            line_width_property_->hide();
            break;
    }
    queueRender();
}

void GridDisplay::updateOffset() {
    if (grid_) {
        grid_->getSceneNode()->setPosition(offset_property_->getVector());
        queueRender();
    }
}

void GridDisplay::updatePlane() {
    if (!grid_) {
        return;
    }
    Ogre::Quaternion orient;
    switch ((Plane)plane_property_->getOptionInt()) {
        case XZ:
            orient = Ogre::Quaternion(1, 0, 0, 0);
            break;
        case YZ:
            orient = Ogre::Quaternion(Ogre::Vector3(0, -1, 0),
                                      Ogre::Vector3(0, 0, 1),
                                      Ogre::Vector3(1, 0, 0));
            break;
        case XY:
        default:
            orient = Ogre::Quaternion(Ogre::Vector3(1, 0, 0),
                                      Ogre::Vector3(0, 0, -1),
                                      Ogre::Vector3(0, 1, 0));
            break;
    }
    grid_->getSceneNode()->setOrientation(orient);

    queueRender();
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz

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

#ifndef AVIZ_RENDERING__OBJECTS__GRID_HPP_
#define AVIZ_RENDERING__OBJECTS__GRID_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <OgreSharedPtr.h>

#include "autonomy/tools/aviz/rendering/visibility_control.hpp"

namespace Ogre {
class SceneManager;
class ManualObject;
class SceneNode;
class Any;
}  // namespace Ogre

namespace aviz {
namespace rendering {

class BillboardLine;

/**
 * @brief Grid rendering object
 * Grid visualization object
 *
 * Displays a grid of cells, drawn with lines.  A grid with an identity orientation is drawn along the XZ plane.
 */
class AVIZ_RENDERING_PUBLIC Grid
{
public:
    enum Style {
        Lines,
        Billboards,
    };

    /**
     * @brief Constructor
     *
     * @param manager The scene manager this object is part of
     * @param parent_node Parent scene node (nullptr for root)
     * @param style Rendering style (Lines or Billboards)
     * @param cell_count The number of cells to draw
     * @param cell_length The size of each cell
     * @param line_width The line width of the cells if it is rendered in Billboards style
     * @param color The color of the lines
     */
    Grid(Ogre::SceneManager* manager, Ogre::SceneNode* parent_node, Style style, uint32_t cell_count, float cell_length,
         float line_width, const Ogre::ColourValue& color);
    ~Grid();

    void create();

    /**
     * @brief Get the Ogre scene node associated with this grid
     *
     * @return The Ogre scene node associated with this grid
     */
    Ogre::SceneNode* getSceneNode() {
        return scene_node_;
    }

    /**
     * @brief Sets user data on all ogre objects we own
     */
    void setUserData(const Ogre::Any& data);

    void setStyle(Style style);
    Style getStyle() {
        return style_;
    }

    void setColor(const Ogre::ColourValue& color);
    Ogre::ColourValue getColor() {
        return color_;
    }

    void setCellCount(uint32_t count);
    uint32_t getCellCount() {
        return cell_count_;
    }

    void setCellLength(float len);
    float getCellLength() {
        return cell_length_;
    }

    void setLineWidth(float width);
    float getLineWidth() {
        return line_width_;
    }

    void setHeight(uint32_t count);
    uint32_t getHeight() {
        return height_count_;
    }

    /// Exposed for testing
    Ogre::ManualObject* getManualObject() {
        return manual_object_;
    }
    std::shared_ptr<BillboardLine> getBillboardLine() {
        return billboard_line_;
    }

private:
    typedef std::function<void(const Ogre::Vector3& p1, const Ogre::Vector3& p2)> AddLineFunction;
    void createBillboardGrid() const;
    void createManualGrid() const;
    void createLines(AddLineFunction addLine) const;
    void createGridPlane(float extent, uint32_t height, AddLineFunction addLine) const;
    void createVerticalLinesBetweenPlanes(float extent, AddLineFunction addLine) const;
    void addBillboardLine(const Ogre::Vector3& p1, const Ogre::Vector3& p2) const;
    void addManualLine(const Ogre::Vector3& p1, const Ogre::Vector3& p2) const;
    uint32_t numberOfVerticalLines() const;

    Ogre::SceneManager* scene_manager_;
    Ogre::SceneNode* scene_node_;        ///< The scene node that this grid is attached to
    Ogre::ManualObject* manual_object_;  ///< The manual object used to draw the grid

    std::shared_ptr<BillboardLine> billboard_line_;

    Ogre::MaterialPtr material_;

    Style style_;
    uint32_t cell_count_;
    float cell_length_;
    float line_width_;
    uint32_t height_count_;
    Ogre::ColourValue color_;
};

}  // namespace rendering
}  // namespace aviz

#endif  // AVIZ_RENDERING__OBJECTS__GRID_HPP_

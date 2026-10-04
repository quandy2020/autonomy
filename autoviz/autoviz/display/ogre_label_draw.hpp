/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_label_draw.hpp
 * @brief Draw helper for 3D text labels (Ogre MovableText or GL overlay).
 *
 * When @c ogre_scene_host is set, labels are created as Ogre MovableText
 * attachments; otherwise the SceneOverlay text path is used.
 *
 * Consumed by @ref TfDisplay (frame names), markers, and other name overlays.
 *
 * @see TextLabelInstance
 * @see rendering::OgreTextLabel
 * @see TfDisplay
 */

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

namespace autoviz {
namespace common {
class DisplayContext;
}  // namespace common
namespace rendering {
class SceneOverlay;
struct OgreTextLabel;
}  // namespace rendering

namespace display {

/**
 * @struct TextLabelInstance
 * @brief One billboard-style text label in world space.
 */
struct TextLabelInstance {
  std::string text;          /**< UTF-8 / Latin-1 label string. */
  QVector3D position;        /**< Anchor point in fixed / world frame. */
  QColor color;              /**< Text color. */
  float char_height = 0.2f;  /**< Character height in world units. */
  float space_width = 0.f;   /**< Extra letter spacing; @c 0 = default. */
};

/**
 * @brief Draws text labels via Ogre MovableText when available, else GL.
 *
 * @param context Display context (Ogre host / view).
 * @param scene GL overlay fallback.
 * @param display_name Stable id for Ogre object naming.
 * @param labels Label instances to draw.
 * @return @c true if a backend accepted the draw.
 *
 * @see TextLabelInstance
 */
bool drawLabelsOgreOrGl(common::DisplayContext* context,
                        rendering::SceneOverlay& scene,
                        const std::string& display_name,
                        const std::vector<TextLabelInstance>& labels);

}  // namespace display
}  // namespace autoviz

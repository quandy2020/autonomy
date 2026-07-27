/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

struct TextLabelInstance {
  std::string text;
  QVector3D position;
  QColor color;
  float char_height = 0.2f;
  float space_width = 0.f;
};

/** Ogre MovableText labels when ogre_scene_host is set. */
bool drawLabelsOgreOrGl(common::DisplayContext* context,
                        rendering::SceneOverlay& scene,
                        const std::string& display_name,
                        const std::vector<TextLabelInstance>& labels);

}  // namespace display
}  // namespace autoviz

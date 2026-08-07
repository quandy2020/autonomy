/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_grid.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <algorithm>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "autoviz/rendering/ogre_material_manager.hpp"

namespace autoviz {
namespace rendering {
namespace {

Ogre::ColourValue ToOgreColor(const QColor& color, float alpha) {
  return Ogre::ColourValue(color.redF(), color.greenF(), color.blueF(), alpha);
}

}  // namespace

OgreGrid::OgreGrid(Ogre::SceneManager* scene_manager,
                   Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager) {
  if (scene_manager_ == nullptr) {
    return;
  }
  Ogre::SceneNode* parent = parent_node != nullptr
                                ? parent_node
                                : scene_manager_->getRootSceneNode();
  scene_node_ = parent->createChildSceneNode("AvizReferenceGridNode");
  manual_object_ = scene_manager_->createManualObject("AvizReferenceGrid");
  scene_node_->attachObject(manual_object_);
  OgreMaterialManager::ensureDefaultMaterials();
  dirty_ = true;
  rebuild();
}

OgreGrid::~OgreGrid() {
  if (scene_manager_ == nullptr) {
    return;
  }
  if (manual_object_ != nullptr) {
    scene_manager_->destroyManualObject(manual_object_);
    manual_object_ = nullptr;
  }
  if (scene_node_ != nullptr) {
    scene_manager_->destroySceneNode(scene_node_);
    scene_node_ = nullptr;
  }
}

void OgreGrid::setSettings(const ReferenceGridSettings& settings) {
  if (settings_ == settings && !dirty_) {
    return;
  }
  settings_ = settings;
  dirty_ = true;
  rebuild();
}

void OgreGrid::setVisible(bool visible) {
  if (visible_ == visible) {
    return;
  }
  visible_ = visible;
  if (scene_node_ != nullptr) {
    scene_node_->setVisible(visible);
  }
}

void OgreGrid::rebuild() {
  if (!dirty_ || manual_object_ == nullptr) {
    return;
  }
  dirty_ = false;
  manual_object_->clear();

  const int cell_count = std::max(1, settings_.plane_cell_count);
  const float step = std::max(0.001f, settings_.cell_length);
  const float extent = static_cast<float>(cell_count) * step * 0.5f;
  const Ogre::ColourValue color =
      ToOgreColor(settings_.color, settings_.alpha);

  Ogre::MaterialPtr material =
      OgreMaterialManager::createMaterialWithNoLighting("Autoviz/LineNoLighting");
  OgreMaterialManager::enableAlphaBlending(material, color.a);

  const int line_pairs = (cell_count + 1) * 2;
  manual_object_->estimateVertexCount(static_cast<size_t>(line_pairs * 2));
  manual_object_->begin(material->getName(), Ogre::RenderOperation::OT_LINE_LIST);

  for (int i = 0; i <= cell_count; ++i) {
    const float p = extent - static_cast<float>(i) * step;
    manual_object_->position(-extent, p, 0.f);
    manual_object_->colour(color);
    manual_object_->position(extent, p, 0.f);
    manual_object_->colour(color);
    manual_object_->position(p, -extent, 0.f);
    manual_object_->colour(color);
    manual_object_->position(p, extent, 0.f);
    manual_object_->colour(color);
  }
  manual_object_->end();
}

}  // namespace rendering
}  // namespace autoviz

#endif

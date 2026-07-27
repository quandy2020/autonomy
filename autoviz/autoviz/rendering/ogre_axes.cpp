/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_axes.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "autoviz/rendering/ogre_material_manager.hpp"

namespace autoviz {
namespace rendering {

OgreAxes::OgreAxes(Ogre::SceneManager* scene_manager,
                   Ogre::SceneNode* parent_node)
    : scene_manager_(scene_manager) {
  if (scene_manager_ == nullptr) {
    return;
  }
  Ogre::SceneNode* parent = parent_node != nullptr
                                ? parent_node
                                : scene_manager_->getRootSceneNode();
  scene_node_ = parent->createChildSceneNode("AvizReferenceAxesNode");
  manual_object_ = scene_manager_->createManualObject("AvizReferenceAxes");
  scene_node_->attachObject(manual_object_);
  OgreMaterialManager::ensureDefaultMaterials();
  dirty_ = true;
  rebuild();
}

OgreAxes::~OgreAxes() {
  if (scene_manager_ == nullptr) {
    return;
  }
  if (manual_object_ != nullptr) {
    scene_manager_->destroyManualObject(manual_object_);
  }
  if (scene_node_ != nullptr) {
    scene_manager_->destroySceneNode(scene_node_);
  }
}

void OgreAxes::setLength(float length) {
  if (length_ == length && !dirty_) {
    return;
  }
  length_ = std::max(0.01f, length);
  dirty_ = true;
  rebuild();
}

void OgreAxes::setVisible(bool visible) {
  visible_ = visible;
  if (scene_node_ != nullptr) {
    scene_node_->setVisible(visible);
  }
}

void OgreAxes::rebuild() {
  if (!dirty_ || manual_object_ == nullptr) {
    return;
  }
  dirty_ = false;
  manual_object_->clear();

  Ogre::MaterialPtr material =
      OgreMaterialManager::createMaterialWithNoLighting("Autoviz/LineNoLighting");
  manual_object_->estimateVertexCount(6);
  manual_object_->begin(material->getName(), Ogre::RenderOperation::OT_LINE_LIST);

  const float l = length_;
  manual_object_->position(0.f, 0.f, 0.f);
  manual_object_->colour(1.f, 0.f, 0.f, 1.f);
  manual_object_->position(l, 0.f, 0.f);
  manual_object_->colour(1.f, 0.f, 0.f, 1.f);
  manual_object_->position(0.f, 0.f, 0.f);
  manual_object_->colour(0.f, 1.f, 0.f, 1.f);
  manual_object_->position(0.f, l, 0.f);
  manual_object_->colour(0.f, 1.f, 0.f, 1.f);
  manual_object_->position(0.f, 0.f, 0.f);
  manual_object_->colour(0.f, 0.f, 1.f, 1.f);
  manual_object_->position(0.f, 0.f, l);
  manual_object_->colour(0.f, 0.f, 1.f, 1.f);
  manual_object_->end();
}

}  // namespace rendering
}  // namespace autoviz

#endif

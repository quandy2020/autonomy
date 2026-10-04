/******************************************************************************
 * Copyright 2008, Willow Garage, Inc.
 * Copyright 2017–2018, Open Source Robotics Foundation, Inc. / Bosch.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_movable_text.hpp"

#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>
#include <OgreMaterialManager.h>
#include <OgreNode.h>
#include <OgrePass.h>
#include <OgreRenderQueue.h>
#include <OgreStringConverter.h>
#include <OgreTechnique.h>

namespace autoviz {
namespace rendering {
namespace {

Ogre::String MakeUniqueName() {
  static unsigned int counter = 0;
  return "AvizMovableText/" + Ogre::StringConverter::toString(++counter);
}

}  // namespace

OgreMovableText::OgreMovableText(const Ogre::String& caption,
                                 const Ogre::String& font_name,
                                 Ogre::Real char_height,
                                 const Ogre::ColourValue& color)
    : caption_(caption),
      font_name_(font_name),
      color_(color),
      char_height_(char_height) {
  mBox.setExtents(Ogre::Vector3::ZERO, Ogre::Vector3::ZERO);
  mParentNode = nullptr;
  setCastShadows(false);
  ensureMaterial();
  // Empty render op — caption rendering is a follow-up (rviz MovableText port).
  mRenderOp.vertexData = OGRE_NEW Ogre::VertexData();
  mRenderOp.vertexData->vertexCount = 0;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  mRenderOp.useIndexes = false;
  mRenderOp.vertexData->vertexDeclaration->addElement(
      0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
}

OgreMovableText::~OgreMovableText() {
  if (mRenderOp.vertexData != nullptr) {
    OGRE_DELETE mRenderOp.vertexData;
    mRenderOp.vertexData = nullptr;
  }
  if (material_) {
    Ogre::MaterialManager::getSingleton().remove(material_->getHandle());
  }
}

void OgreMovableText::ensureMaterial() {
  if (material_) {
    return;
  }
  material_ = Ogre::MaterialManager::getSingleton().create(
      MakeUniqueName() + "/Mat",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setLightingEnabled(false);
  material_->setDepthCheckEnabled(false);
  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
}

void OgreMovableText::setCaption(const Ogre::String& caption) {
  caption_ = caption;
}

void OgreMovableText::setColor(const Ogre::ColourValue& color) {
  color_ = color;
}

void OgreMovableText::setCharacterHeight(Ogre::Real height) {
  char_height_ = height;
}

void OgreMovableText::setSpaceWidth(Ogre::Real width) { space_width_ = width; }

void OgreMovableText::setTextAlignment(HorizontalAlignment horizontal,
                                       VerticalAlignment vertical) {
  horizontal_alignment_ = horizontal;
  vertical_alignment_ = vertical;
}

void OgreMovableText::getWorldTransforms(Ogre::Matrix4* xform) const {
  if (mParentNode != nullptr) {
    *xform = mParentNode->_getFullTransform();
  } else {
    *xform = Ogre::Matrix4::IDENTITY;
  }
}

void OgreMovableText::_notifyCurrentCamera(Ogre::Camera* camera) {
  Ogre::SimpleRenderable::_notifyCurrentCamera(camera);
}

void OgreMovableText::_updateRenderQueue(Ogre::RenderQueue* queue) {
  if (mRenderOp.vertexData != nullptr && mRenderOp.vertexData->vertexCount > 0) {
    queue->addRenderable(this, mRenderQueueID, OGRE_RENDERABLE_DEFAULT_PRIORITY);
  }
}

const Ogre::String& OgreMovableText::getMovableType() const {
  static Ogre::String type = "AvizMovableText";
  return type;
}

}  // namespace rendering
}  // namespace autoviz


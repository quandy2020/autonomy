/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/common/overlay_object.hpp"

#include <cstring>

#include <OgreMaterialManager.h>
#include <OgreOverlay.h>
#include <OgreOverlayContainer.h>
#include <OgreOverlayManager.h>
#include <OgrePanelOverlayElement.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include <glog/logging.h>

namespace autoviz {
namespace display {
namespace viz {

ScopedPixelBuffer::ScopedPixelBuffer(
    Ogre::HardwarePixelBufferSharedPtr pixel_buffer)
    : pixel_buffer_(std::move(pixel_buffer)) {
  if (pixel_buffer_) {
    pixel_buffer_->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  }
}

ScopedPixelBuffer::~ScopedPixelBuffer() {
  if (pixel_buffer_) {
    pixel_buffer_->unlock();
  }
}

Ogre::HardwarePixelBufferSharedPtr ScopedPixelBuffer::getPixelBuffer() const {
  return pixel_buffer_;
}

QImage ScopedPixelBuffer::getQImage(unsigned int width, unsigned int height) {
  if (!pixel_buffer_) {
    return QImage();
  }
  const Ogre::PixelBox& pixel_box = pixel_buffer_->getCurrentLock();
  auto* dest = static_cast<Ogre::uint8*>(pixel_box.data);
  std::memset(dest, 0, static_cast<size_t>(width) * height * 4U);
  return QImage(dest, static_cast<int>(width), static_cast<int>(height),
                QImage::Format_ARGB32);
}

QImage ScopedPixelBuffer::getQImage(unsigned int width, unsigned int height,
                                    QColor& bg_color) {
  QImage hud = getQImage(width, height);
  if (hud.isNull()) {
    return hud;
  }
  hud.fill(bg_color.rgba());
  return hud;
}

OverlayObject::OverlayObject(const std::string& name) : name_(name) {
  const std::string material_name = name_ + "Material";
  Ogre::OverlayManager* overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
  if (overlay_mgr == nullptr) {
    LOG(ERROR) << "OverlayObject: Ogre::OverlayManager not available";
    return;
  }
  overlay_ = overlay_mgr->create(name_);
  panel_ = static_cast<Ogre::PanelOverlayElement*>(
      overlay_mgr->createOverlayElement("Panel", name_ + "Panel"));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);

  panel_material_ = Ogre::MaterialManager::getSingleton().create(
      material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  panel_->setMaterialName(panel_material_->getName());
  overlay_->add2D(panel_);
}

OverlayObject::~OverlayObject() {
  hide();
  if (panel_material_) {
    panel_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());
  }
}

std::string OverlayObject::getName() const { return name_; }

void OverlayObject::hide() {
  if (overlay_ != nullptr && overlay_->isVisible()) {
    overlay_->hide();
  }
}

void OverlayObject::show() {
  if (overlay_ != nullptr && !overlay_->isVisible()) {
    overlay_->show();
  }
}

bool OverlayObject::isTextureReady() const { return texture_ != nullptr; }

void OverlayObject::updateTextureSize(unsigned int width, unsigned int height) {
  if (panel_material_ == nullptr || panel_ == nullptr) {
    return;
  }
  const std::string texture_name = name_ + "Texture";
  if (width == 0) {
    LOG(WARNING) << "[OverlayObject] width=0 requested; clamping to 1";
    width = 1;
  }
  if (height == 0) {
    LOG(WARNING) << "[OverlayObject] height=0 requested; clamping to 1";
    height = 1;
  }
  if (!isTextureReady() || texture_->getWidth() != width ||
      texture_->getHeight() != height) {
    if (isTextureReady()) {
      Ogre::TextureManager::getSingleton().remove(texture_name);
      panel_material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
    }
    texture_ = Ogre::TextureManager::getSingleton().createManual(
        texture_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_A8R8G8B8,
        Ogre::TU_DEFAULT);
    panel_material_->getTechnique(0)->getPass(0)->createTextureUnitState(
        texture_name);
    panel_material_->getTechnique(0)->getPass(0)->setSceneBlending(
        Ogre::SBT_TRANSPARENT_ALPHA);
  }
}

ScopedPixelBuffer OverlayObject::getBuffer() {
  if (isTextureReady()) {
    return ScopedPixelBuffer(texture_->getBuffer());
  }
  return ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr());
}

void OverlayObject::setPosition(double left, double top) {
  if (panel_ != nullptr) {
    panel_->setPosition(static_cast<Ogre::Real>(left),
                        static_cast<Ogre::Real>(top));
  }
}

void OverlayObject::setDimensions(double width, double height) {
  if (panel_ != nullptr) {
    panel_->setDimensions(static_cast<Ogre::Real>(width),
                          static_cast<Ogre::Real>(height));
  }
}

bool OverlayObject::isVisible() const {
  return overlay_ != nullptr && overlay_->isVisible();
}

unsigned int OverlayObject::getTextureWidth() const {
  return isTextureReady() ? texture_->getWidth() : 0U;
}

unsigned int OverlayObject::getTextureHeight() const {
  return isTextureReady() ? texture_->getHeight() : 0U;
}

}  // namespace viz
}  // namespace display
}  // namespace autoviz

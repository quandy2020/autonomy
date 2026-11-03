/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

/**
 * @file overlay_object.hpp
 * @brief Ogre Overlay panel + ARGB texture helpers for HUD displays.
 */

#pragma once

#include <memory>
#include <string>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>

#include <QColor>
#include <QImage>

namespace Ogre {
class Overlay;
class PanelOverlayElement;
}  // namespace Ogre

namespace autoviz {
namespace display {
namespace viz {

/**
 * @class ScopedPixelBuffer
 * @brief RAII lock around an Ogre hardware pixel buffer for QImage painting.
 */
class ScopedPixelBuffer {
 public:
  explicit ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);
  ~ScopedPixelBuffer();

  ScopedPixelBuffer(const ScopedPixelBuffer&) = delete;
  ScopedPixelBuffer& operator=(const ScopedPixelBuffer&) = delete;

  Ogre::HardwarePixelBufferSharedPtr getPixelBuffer() const;
  QImage getQImage(unsigned int width, unsigned int height);
  QImage getQImage(unsigned int width, unsigned int height, QColor& bg_color);

 private:
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer_;
};

/**
 * @class OverlayObject
 * @brief Named Ogre Overlay panel that displays a mutable ARGB texture.
 *
 * Instantiate after Ogre OverlaySystem is ready (typically first enable of a
 * overlay display). Paint via @ref getBuffer into a @c QImage wrapping the
 * locked pixel buffer, then @ref setPosition / @ref setDimensions.
 */
class OverlayObject {
 public:
  using Ptr = std::shared_ptr<OverlayObject>;

  explicit OverlayObject(const std::string& name);
  ~OverlayObject();

  OverlayObject(const OverlayObject&) = delete;
  OverlayObject& operator=(const OverlayObject&) = delete;

  std::string getName() const;
  void hide();
  void show();
  bool isTextureReady() const;
  void updateTextureSize(unsigned int width, unsigned int height);
  ScopedPixelBuffer getBuffer();
  void setPosition(double left, double top);
  void setDimensions(double width, double height);
  bool isVisible() const;
  unsigned int getTextureWidth() const;
  unsigned int getTextureHeight() const;

 private:
  const std::string name_;
  Ogre::Overlay* overlay_ = nullptr;
  Ogre::PanelOverlayElement* panel_ = nullptr;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;
};

}  // namespace viz
}  // namespace display
}  // namespace autoviz

/******************************************************************************
 * Copyright 2008, Willow Garage, Inc.
 * Copyright 2017–2018, Open Source Robotics Foundation, Inc. / Bosch.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file ogre_movable_text.hpp
 * @brief Billboarding caption (rviz MovableText subset).
 *
 * Minimal stub sufficient for @ref OgreSceneHost::setDisplayLabels; geometry
 * update can be filled in later from rviz_rendering::MovableText.
 */

#pragma once

#include <OgreColourValue.h>
#include <OgreSimpleRenderable.h>
#include <OgreVector.h>

namespace Ogre {
class Camera;
class Font;
class RenderQueue;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

class OgreMovableText : public Ogre::SimpleRenderable {
 public:
  enum HorizontalAlignment { H_LEFT, H_CENTER };
  enum VerticalAlignment { V_BELOW, V_ABOVE, V_CENTER };

  explicit OgreMovableText(
      const Ogre::String& caption,
      const Ogre::String& font_name = "Liberation Sans",
      Ogre::Real char_height = 1.0,
      const Ogre::ColourValue& color = Ogre::ColourValue::White);
  ~OgreMovableText() override;

  void setCaption(const Ogre::String& caption);
  void setColor(const Ogre::ColourValue& color);
  void setCharacterHeight(Ogre::Real height);
  void setSpaceWidth(Ogre::Real width);
  void setTextAlignment(HorizontalAlignment horizontal,
                        VerticalAlignment vertical);

  Ogre::Real getBoundingRadius() const override { return radius_; }
  Ogre::Real getSquaredViewDepth(const Ogre::Camera* /*cam*/) const override {
    return 0;
  }
  const Ogre::MaterialPtr& getMaterial() const override { return material_; }
  void getWorldTransforms(Ogre::Matrix4* xform) const override;
  void _notifyCurrentCamera(Ogre::Camera* camera) override;
  void _updateRenderQueue(Ogre::RenderQueue* queue) override;
  const Ogre::String& getMovableType() const override;

 private:
  void ensureMaterial();

  Ogre::String caption_;
  Ogre::String font_name_;
  HorizontalAlignment horizontal_alignment_ = H_CENTER;
  VerticalAlignment vertical_alignment_ = V_CENTER;
  Ogre::ColourValue color_;
  Ogre::Real char_height_ = 1.f;
  Ogre::Real space_width_ = 0.f;
  Ogre::Real radius_ = 0.f;
  Ogre::MaterialPtr material_;
};

}  // namespace rendering
}  // namespace autoviz


/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Open Source Robotics
 * Foundation, Inc. · Adapted for Autoviz (BSD-3-Clause).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND; see the
 * RViz BSD-3-Clause license for the full disclaimer.
 *****************************************************************************/

// Adapted from: http://www.ogre3d.org/wiki/index.php/MovableText
//          now: http://www.ogre3d.org/tikiwiki/tiki-index.php?page=MovableText
// Original authors:
/*
 * File: MovableText.h
 *
 * description: This creates a billboarding object that displays a text.
 *
 * @author  2003 by cTh see gavocanov@rambler.ru
 * @update  2006 by barraq see nospam@barraquand.com
 */

#pragma once

#include <cassert>

#include <OgreMovableObject.h>
#include <OgreQuaternion.h>
#include <OgreRenderable.h>
#include <OgreSharedPtr.h>
#include <OgreSimpleRenderable.h>
#include <OgreVector.h>


namespace Ogre {
class Camera;
class Font;
class RenderQueue;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

class OgreMovableText : public Ogre::SimpleRenderable
{
public:
  enum HorizontalAlignment
  {
    H_LEFT, H_CENTER
  };
  enum VerticalAlignment
  {
    V_BELOW, V_ABOVE, V_CENTER
  };

public:
  explicit OgreMovableText(
    const Ogre::String & caption,
    const Ogre::String & fontName = "Liberation Sans",
    Ogre::Real charHeight = 1.0,
    const Ogre::ColourValue & color = Ogre::ColourValue::White);
  ~OgreMovableText() override;
  void setFontName(const Ogre::String & font_name);
  void setCaption(const Ogre::String & caption);
  void setColor(const Ogre::ColourValue & color);
  void setCharacterHeight(Ogre::Real height);
  void setLineSpacing(Ogre::Real height);
  void setSpaceWidth(Ogre::Real width);
  void setTextAlignment(
    const HorizontalAlignment & horizontal_alignment,
    const VerticalAlignment & vertical_alignment);
  void setGlobalTranslation(Ogre::Vector3 translation);
  void setLocalTranslation(Ogre::Vector3 translation);
  void showOnTop(bool show = true);
  const Ogre::String & getFontName() const
  {
    return font_name_;
  }
  const Ogre::String & getCaption() const
  {
    return caption_;
  }
  const Ogre::ColourValue & getColor() const
  {
    return color_;
  }
  Ogre::Real getCharacterHeight() const
  {
    return char_height_;
  }
  Ogre::Real getSpaceWidth() const
  {
    return space_width_;
  }
  Ogre::Vector3 getGlobalTranslation() const
  {
    return global_translation_;
  }
  Ogre::Vector3 getLocalTranslation() const
  {
    return local_translation_;
  }
  bool getShowOnTop() const
  {
    return on_top_;
  }
  const Ogre::AxisAlignedBox & getBoundingBox() const override
  {
    return mBox;
  }
  Ogre::Real getBoundingRadius() const override
  {
    return radius_;
  }
  const Ogre::MaterialPtr & getMaterial() const override
  {
    assert(material_);
    return material_;
  }
  void
  visitRenderables(Ogre::Renderable::Visitor * visitor, bool debug_renderables) override;
  void update();

protected:
  void setupGeometry();
  unsigned int calculateVertexCount() const;
  void setupRenderOperation();
  Ogre::HardwareVertexBufferSharedPtr setupHardwareBuffers() const;
  void calculateTotalDimensionsForPositioning(float & total_height, float & total_width) const;
  float getVerticalStartFromVerticalAlignment(float total_height) const;
  float getLineStartFromHorizontalAlignment(float total_width) const;
  void fillVertexBuffer(
    Ogre::HardwareVertexBufferSharedPtr & position_and_texture_buffer,
    float top, float starting_left);

  void updateColors();

  void getWorldTransforms(Ogre::Matrix4 * xform) const override;
  Ogre::Real getSquaredViewDepth(const Ogre::Camera * cam) const override
  {
    (void) cam;
    return 0;
  }
  const Ogre::Quaternion & getWorldOrientation() const;
  const Ogre::Vector3 & getWorldPosition() const;
  const Ogre::String & getMovableType() const override
  {
    static Ogre::String movType = "AvizMovableText";
    return movType;
  }

  void _notifyCurrentCamera(Ogre::Camera * camera) override;
  void _updateRenderQueue(Ogre::RenderQueue * queue) override;

  const Ogre::LightList & getLights() const override
  {
    return light_list_;
  }

  void fillColorBuffer(Ogre::RGBA color) const;

  void getRenderOperation(Ogre::RenderOperation & op) override;

private:
  Ogre::String font_name_;
  Ogre::String name_;
  Ogre::String caption_;
  HorizontalAlignment horizontal_alignment_;
  VerticalAlignment vertical_alignment_;

  Ogre::ColourValue color_;
  Ogre::Real radius_;

  Ogre::Real char_height_;
  Ogre::Real line_spacing_;
  Ogre::Real space_width_;

  bool needs_update_ = true;
  bool needs_color_update_;
  bool on_top_;

  Ogre::Vector3 global_translation_;
  Ogre::Vector3 local_translation_;

  Ogre::Font * font_;
  Ogre::MaterialPtr material_;

  Ogre::LightList light_list_;
};

}  // namespace rendering
}  // namespace autoviz


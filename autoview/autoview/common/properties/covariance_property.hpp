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

#pragma once

#include <deque>
#include <memory>

#include <OgreColourValue.h>

#include <QColor>  // NOLINT cpplint cannot handle include order here

#include "autoview/common/properties/bool_property.hpp"

namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace autoview {
namespace common {
namespace properties {

namespace rendering
{
struct CovarianceUserData;
}

namespace autoview {
namespace common {
namespace properties {

class Property;
class ColorProperty;
class FloatProperty;
class EnumProperty;


/** @brief Property specialized for covariance visuals. */
class CovarianceProperty : public BoolProperty
{
  Q_OBJECT

public:
  explicit CovarianceProperty(
    const QString & name = "Covariance",
    bool default_value = false,
    const QString & description = QString(),
    Property * parent = nullptr,
    const char * changed_slot = nullptr);

  ~CovarianceProperty() override = default;

  /**
   * @brief Return all property data in an object understood by covariance_visual
   */
  rendering::CovarianceUserData getUserData();

private Q_SLOTS:
  void updateColorStyleChoice();

private:
  BoolProperty* position_property_;
  ColorProperty* position_color_property_;
  FloatProperty* position_alpha_property_;
  FloatProperty* position_scale_property_;
  BoolProperty* orientation_property_;
  EnumProperty* orientation_frame_property_;
  EnumProperty* orientation_colorstyle_property_;
  ColorProperty* orientation_color_property_;
  FloatProperty* orientation_alpha_property_;
  FloatProperty* orientation_offset_property_;
  FloatProperty* orientation_scale_property_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
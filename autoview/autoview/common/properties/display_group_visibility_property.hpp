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

#include "autoview/common/properties/display_visibility_property.hpp"

#include <cstdint>
#include <map>
#include <string>

#include <QObject>  // NOLINT: cpplint cannot handle the include order here
#include <QString>  // NOLINT: cpplint cannot handle the include order here


namespace autoview {
namespace common {
namespace properties {

class DisplayContext;
class DisplayGroup;
class Display;

namespace properties
{

class Property;
class BoolProperty;
class BoolProperty;
class DisplayVisibilityProperty;

/*
 * Manages the visibility of all displays in a display group
 * by switching one bit in Ogre's visibility mask.
 */
class RVIZ_COMMON_PUBLIC DisplayGroupVisibilityProperty : public DisplayVisibilityProperty
{
  Q_OBJECT

public:
  /* @param parent Parent display (will not get a visibility property)
   * @param context The display context
   * @param root_property Where to attach new properties to
   */
  DisplayGroupVisibilityProperty(
    uint32_t vis_bit,
    DisplayGroup * display_group,
    Display * parent_display,
    const QString & name = QString(),
    bool default_value = false,
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  virtual ~DisplayGroupVisibilityProperty();

  // @brief Update visibility masks of all objects in the Ogre scene
  virtual void update();

public Q_SLOTS:
  void onDisplayAdded(Display * display);
  void onDisplayRemoved(Display * display);

private:
  // sort the properties in the same way as in the
  // root display group
  void sortDisplayList();

  DisplayGroup * display_group_;
  std::map<Display *, DisplayVisibilityProperty *> disp_vis_props_;
  Display * parent_display_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
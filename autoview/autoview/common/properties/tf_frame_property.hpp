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

#include <string>
#include "autoview/common/frame_manager_iface.hpp"
#include "autoview/common/properties/editable_enum_property.hpp"

namespace autoview {

class FrameManager;

namespace common {
namespace properties {

class TfFrameProperty : public EditableEnumProperty
{
  Q_OBJECT

public:
  TfFrameProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & description = QString(),
    Property * parent = 0,
    FrameManager * frame_manager = 0,
    bool include_fixed_frame_string = false,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  /// Override from Property to resolve the frame name on the way in.
  virtual bool setValue(const QVariant & new_value);

  /// Get the frame as a QString.
  QString getFrame() const;

  /// Get the frame as a std::string.
  std::string getFrameStd() const;

  /// Set the frame manager.
  void setFrameManager(FrameManagerIface * frame_manager);

  /// Get the frame manager.
  FrameManagerIface * getFrameManager() const;

  static const QString FIXED_FRAME_STRING;

private Q_SLOTS:
  /// Fill the frame list.
  void fillFrameList();

  /// Notify users of changed frame, if required.
  /**
   * If this property is currently set to FIXED_FRAME_STRING, this emits
   * changed() to let users know that a call to getFrame() will now return
   * something different.
   */
  void handleFixedFrameChange();

private:
  FrameManagerIface * frame_manager_;
  bool include_fixed_frame_string_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
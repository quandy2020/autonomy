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

 #include "autoview/common/properties/tf_frame_property.hpp"

#include <algorithm> 
#include <string>
#include <vector>

// #include "tf2_ros/transform_listener.h"

#include "autoview/common/frame_manager.hpp"
    
namespace autoview {
namespace common {
namespace properties {

const QString TfFrameProperty::FIXED_FRAME_STRING = "<Fixed Frame>";

TfFrameProperty::TfFrameProperty(
  const QString & name,
  const QString & default_value,
  const QString & description,
  Property * parent,
  FrameManager * frame_manager,
  bool include_fixed_frame_string,
  const char * changed_slot,
  QObject * receiver)
: EditableEnumProperty(name, default_value, description, parent, changed_slot, receiver),
  frame_manager_(nullptr),
  include_fixed_frame_string_(include_fixed_frame_string)
{
  // Parent class EditableEnumProperty has requestOptions() signal.
  connect(
    this, SIGNAL(requestOptions(EditableEnumProperty*)),
    this, SLOT(fillFrameList()));
  setFrameManager(frame_manager);
}

bool TfFrameProperty::setValue(const QVariant & new_value)
{
  QString new_string = new_value.toString();
  if (new_string.size() > 0 && new_string[0] == '/') {
    new_string = new_string.right(new_string.size() - 1);
  }
  bool result = EditableEnumProperty::setValue(new_string);

  return result;
}

void TfFrameProperty::setFrameManager(FrameManagerIface * frame_manager)
{
  if (frame_manager_ && include_fixed_frame_string_) {
    disconnect(
      frame_manager_, SIGNAL(fixedFrameChanged()),
      this, SLOT(handleFixedFrameChange()));
  }
  frame_manager_ = frame_manager;
  if (frame_manager_ && include_fixed_frame_string_) {
    connect(
      frame_manager_, SIGNAL(fixedFrameChanged()),
      this, SLOT(handleFixedFrameChange()));
  }
}

FrameManagerIface * TfFrameProperty::getFrameManager() const
{
  return frame_manager_;
}

void TfFrameProperty::fillFrameList()
{
  std::vector<std::string> std_frames = frame_manager_->getAllFrameNames();
  std::sort(std_frames.begin(), std_frames.end() );

  clearOptions();
  if (include_fixed_frame_string_) {
    addOption(FIXED_FRAME_STRING);
  }
  for (size_t i = 0; i < std_frames.size(); i++) {
    addOptionStd(std_frames[i]);
  }
}

QString TfFrameProperty::getFrame() const
{
  QString frame = getValue().toString();
  if (frame == FIXED_FRAME_STRING && frame_manager_) {
    return QString::fromStdString(frame_manager_->getFixedFrame() );
  }
  return frame;
}

std::string TfFrameProperty::getFrameStd() const
{
  return getFrame().toStdString();
}

void TfFrameProperty::handleFixedFrameChange()
{
  if (getValue().toString() == FIXED_FRAME_STRING) {
    Q_EMIT changed();
  }
}

}  // namespace properties
}  // namespace common
}  // namespace autoview

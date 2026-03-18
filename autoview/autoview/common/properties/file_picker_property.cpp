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

#include "autoview/common/properties/file_picker_property.hpp"
#include "autoview/common/properties/file_picker.hpp"

namespace autoview {
namespace common {
namespace properties {

FilePickerProperty::FilePickerProperty(
  const QString & name,
  const QString & default_value,
  const QString & description,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: StringProperty(name, default_value, description, parent, changed_slot, receiver)
{}

QWidget * FilePickerProperty::createEditor(QWidget * parent, const QStyleOptionViewItem & option)
{
  (void) option;
  auto file_picker = new FilePicker(this, parent);
  file_picker->setFrame(false);
  return file_picker;
}

}  // namespace properties
}  // namespace common
}  // namespace autoview

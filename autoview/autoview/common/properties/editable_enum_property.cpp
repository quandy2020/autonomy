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

#include "autoview/common/properties/editable_enum_property.hpp"

#include <QCompleter>

#include <string>

#include "autoview/common/properties/editable_combo_box.hpp"

namespace autoview {
namespace common {
namespace properties {

EditableEnumProperty::EditableEnumProperty(
  const QString & name,
  const QString & default_value,
  const QString & description,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: StringProperty(name, default_value, description, parent, changed_slot, receiver)
{
}

void EditableEnumProperty::clearOptions()
{
  strings_.clear();
}

void EditableEnumProperty::addOption(const QString & option)
{
  strings_.push_back(option);
}

void EditableEnumProperty::addOptionStd(const std::string & option)
{
  addOption(QString::fromStdString(option));
}

QWidget * EditableEnumProperty::createEditor(QWidget * parent, const QStyleOptionViewItem & option)
{
  Q_UNUSED(option);

  // Emit requestOptions() to give listeners a chance to change the option list.
  Q_EMIT requestOptions(this);

  EditableComboBox * cb = new EditableComboBox(parent);
  cb->addItems(strings_);
  cb->setEditText(getValue().toString() );
  QObject::connect(
    cb, SIGNAL(currentIndexChanged(const QString&)), this,
    SLOT(setString(const QString&)));

  // TODO(unknown): need to better handle string value which is not in list.
  return cb;
}

void EditableEnumProperty::sortOptions()
{
  strings_.sort();
}

void EditableEnumProperty::setString(const QString & str)
{
  setValue(str);
}

}  // namespace properties
}  // namespace common
}  // namespace autoview

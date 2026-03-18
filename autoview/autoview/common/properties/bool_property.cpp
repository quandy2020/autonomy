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

#include "autoview/common/properties/bool_property.hpp"

#include <QColor>

namespace autoview {
namespace common {
namespace properties {

BoolProperty::BoolProperty(
  const QString & name,
  bool default_value,
  const QString & description,
  Property * parent,
  const char * changed_slot,
  QObject * receiver)
: Property(name, default_value, description, parent, changed_slot, receiver),
  disable_children_if_false_(false)
{
}

BoolProperty::~BoolProperty() = default;

bool BoolProperty::getBool() const
{
  return getValue().toBool();
}

void BoolProperty::setDisableChildrenIfFalse(bool disable)
{
  disable_children_if_false_ = disable;
}

bool BoolProperty::getDisableChildrenIfFalse()
{
  return disable_children_if_false_;
}

bool BoolProperty::getDisableChildren()
{
  if (disable_children_if_false_) {
    return !getBool() || Property::getDisableChildren();
  }
  return Property::getDisableChildren();
}

bool BoolProperty::setBool(bool value)
{
  return setValue(value);
}

}  // namespace properties
}  // namespace common
}  // namespace autoview

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


#include <memory>

#include "autoview/common/properties/grouped_checkbox_property.hpp"
#include "autoview/common/properties/grouped_checkbox_property_group.hpp"

namespace autoview {
namespace common {
namespace properties {

GroupedCheckboxProperty::GroupedCheckboxProperty(
  std::shared_ptr<GroupedCheckboxPropertyGroup> group,
  const QString & name,
  bool default_value,
  const QString & description,
  Property * parent,
  const char * changed_slot,
  QObject * receiver
)
: BoolProperty(name, default_value, description, parent, changed_slot, receiver), group_(group)
{
  group->addProperty(this);
}

bool GroupedCheckboxProperty::setValue(const QVariant & new_value)
{
  Q_UNUSED(new_value);
  return true;
}

bool GroupedCheckboxProperty::setRawValue(const QVariant & new_value)
{
  return Property::setValue(new_value);
}

void GroupedCheckboxProperty::checkPropertyInGroup()
{
  group_->setChecked(this);
}

}  // namespace properties
}  // namespace common
}  // namespace autoview

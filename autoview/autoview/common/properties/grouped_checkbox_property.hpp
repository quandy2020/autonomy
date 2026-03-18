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

#include <memory>

#include "autoview/common/properties/bool_property.hpp"

namespace autoview {
namespace common {
namespace properties {

class GroupedCheckboxPropertyGroup;

/// Checkbox property that is grouped together with other checkbox properties.
/**
 * Behaves like radio buttons (exclusive selection, no manual deselect).
 */
class GroupedCheckboxProperty : public BoolProperty
{
public:
  explicit
  GroupedCheckboxProperty(
    std::shared_ptr<GroupedCheckboxPropertyGroup> group,
    const QString & name = QString(),
    bool default_value = false,
    const QString & description = QString(),
    Property * parent = nullptr,
    const char * changed_slot = nullptr,
    QObject * receiver = nullptr);

  /// An override for Property::setValue that is a noop.
  /**
   * To set the value correctly, use setBoolValue.
   *
   * This is a workaround for the issue where sometimes, clicking on the
   * GroupedCheckboxPropertyGroup does not send the clicked() event of
   * QTreeView but only the clicked() event of this property, resulting
   * in weird behaviour.
   *
   * \param new_value IGNORED, radio buttons can generally not be deselected
   *   manually.
   */
  bool
  setValue(const QVariant & new_value) override;

  /// Set the value of the underlying property without triggering the group.
  bool
  setRawValue(const QVariant & new_value);

  /// Notify the group so that all other GroupedCheckboxProperty instances are disabled.
  void
  checkPropertyInGroup();

private:
  std::shared_ptr<GroupedCheckboxPropertyGroup> group_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
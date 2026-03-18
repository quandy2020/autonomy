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

#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace autoview {
namespace common {
namespace properties {

class FilePickerProperty : public StringProperty
{
public:
  explicit FilePickerProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & description = QString(),
    Property * parent = nullptr,
    const char * changed_slot = nullptr,
    QObject * receiver = nullptr);

  QWidget * createEditor(QWidget * parent, const QStyleOptionViewItem & option) override;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview

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

#include "autoview/common/properties/property.hpp"

namespace autoview {
namespace common {
namespace properties {

/// Property specialized for string values.
class StringProperty : public Property
{
  Q_OBJECT

public:
  explicit StringProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  /// Get value as std::string.
  std::string getStdString() const;

  /// Get value as QString.
  QString getString() const;

public Q_SLOTS:
  /// Set value from std::string.
  bool setStdString(const std::string & std_str);

  /// Set value from QString.
  bool setString(const QString & str);
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
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

#include <QHash>
#include <QString>

#include "autoview/common/properties/status_property.hpp"

namespace autoview {
namespace common {
namespace properties {


class StatusList : public StatusProperty
{
  Q_OBJECT

public:
  explicit StatusList(const QString & name = QString("Status"), Property * parent = 0);

  virtual void setLevel(Level level);

  /// Add and set a status to the list by name.
  void setStatus(Level level, const QString & name, const QString & text);

  /// Delete a status by name.
  void deleteStatus(const QString & name);

  /// Clear all statuses from the list.
  void clear();

  /// Update the level of the list based on the contained statuses.
  void updateLevel();

  /// Set the prefix of the name for added statuses.
  /**
   * Setting the name to "Foo" will give a displayed name like
   * "Foo: Ok" or "Foo: Error".
   */
  virtual void setName(const QString & name);

private:
  /// Update the label text based on the name_prefix_ and the current status level.
  void updateLabel();

  QHash<QString, StatusProperty *> status_children_;
  QString name_prefix_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
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

#include <QString>
#include <QStringList>
#include <QWidget>

#include "autoview/common/properties/string_property.hpp"

namespace autoview {
namespace common {
namespace properties {

/// Editable Enum property.
/**
 * An editable enum property works like a string property, but with
 * the addition of a drop-down list of predefined choices.
 */
class EditableEnumProperty : public StringProperty
{
  Q_OBJECT

public:
  EditableEnumProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  /// Clear options.
  virtual void clearOptions();

  /// Add an option from a QString.
  virtual void addOption(const QString & option);

  /// Add an option from a std::string.
  void addOptionStd(const std::string & option);

  /// Create the editor.
  virtual QWidget * createEditor(QWidget * parent, const QStyleOptionViewItem & option);
  // virtual QWidget * createEditor(QWidget * parent);

  /// Sort the option strings.
  void sortOptions();

public Q_SLOTS:
  /// Set the string value from a QString.
  virtual void setString(const QString & str);

Q_SIGNALS:
  /// requestOptions() is emitted each time createEditor() is called.
  /**
   * A connection to this signal should never be made with a queued connection,
   * because then the "emit" would return before the changes to the options in
   * the EnumProperty were made.
   *
   * A connected slot should make calls to clearOptions() and/or addOption() as
   * needed.
   * The option list in the EditableEnumProperty will not be cleared before the
   * signal is emitted.
   */
  void requestOptions(EditableEnumProperty * property_in_need_of_options);

protected:
  QStringList strings_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
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

#include <QStringList>

#include <string>

#include "autoview/common/properties/string_property.hpp"

namespace autoview {
namespace common {
namespace properties {


/// Enum property.
/**
 * An enum property works like a string property all the way through
 * the system property system, except when you get a changed() signal
 * you can call getOptionInt() to get the integer value of the current
 * option.
 * The integer returned will be that passed to addOption() for with the
 * string that is currently selected.
 */
class EnumProperty : public StringProperty
{
  Q_OBJECT

public:
  explicit
  EnumProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  /// Clear the list of options.
  /**
   * Does not change the current value of the property.
   */
  virtual
  void
  clearOptions();

  virtual
  void
  addOption(const QString & option, int value = 0);

  void
  addOptionStd(const std::string & option, int value = 0);

  /// Return the int value of the currently-chosen option, or 0 if the current value is not an int.
  virtual int getOptionInt();

  QWidget * createEditor(QWidget * parent, const QStyleOptionViewItem & option) override;

public Q_SLOTS:
  /// Set the value of this property to the given string.
  /**
   * Does not force the value to be one of the list of options.
   */
  virtual void setString(const QString & str);

  /// Set the value of this property to the given std::string.
  /**
   * Does not force the value to be one of the list of options.
   */
  void setStringStd(const std::string & str);

  /// Sort the option strings.
  /**
   * Does not change string/int associations.
   */
  void sortOptions();

Q_SIGNALS:
  /// requestOptions() is emitted each time createEditor() is called.
  /**
   * A connection to this signal should never be made with a queued
   * connection, because then the "emit" would return before the
   * changes to the options in the EnumProperty were made.
   *
   * A connected slot should make calls to clearOptions() and/or
   * addOption() as needed.  The option list in the EnumProperty will
   * not be cleared before the signal is emitted.
   */
  void requestOptions(rviz_common::properties::EnumProperty * property_in_need_of_options);

private:
  QStringList strings_;
  QHash<QString, int> ints_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
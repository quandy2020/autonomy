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

#include "autoview/common/properties/property.hpp"

namespace autoview {
namespace common {
namespace properties {

/// Property specialized to enforce floating point max/min.
class FloatProperty : public Property
{
  Q_OBJECT

public:
  explicit FloatProperty(
    const QString & name = QString(),
    float default_value = 0,
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  /// Set the new value for this property; return true if different, else false.
  /**
   * If the new value is different from the old value, this emits
   * aboutToChange() before changing the value and changed() after.
   *
   * Overridden from Property::setValue() to enforce minimum and maximum.
   */
  bool setValue(const QVariant & new_value) override;

  /// Get the value of the Property as a float.
  float getFloat() const;

  /// Set the enforced minimum value.
  void setMin(float min);

  /// Get the currently enforced minimum.
  float getMin();

  /// Set the enforced maximum value.
  void setMax(float max);

  /// Get the currently enforced maximum.
  float getMax();

public Q_SLOTS:
  /// Float-typed "SLOT" version of setValue().
  bool setFloat(float new_value);

  /// Add the given @a delta to the property value.
  bool add(float delta);

  /// Multiply the property value by the given factor.
  bool multiply(float factor);

private:
  float min_;
  float max_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
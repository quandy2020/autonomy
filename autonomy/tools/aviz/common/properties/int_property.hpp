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

#ifndef AVIZ_COMMON__PROPERTIES__INT_PROPERTY_HPP_
#define AVIZ_COMMON__PROPERTIES__INT_PROPERTY_HPP_

#include <climits>  // for INT_MIN and INT_MAX

#include "autonomy/tools/aviz/common/properties/property.hpp"

namespace aviz {
namespace common {
namespace properties {

/// Property specialized to provide max/min enforcement for integers.
class IntProperty : public Property
{
    Q_OBJECT

public:
    explicit IntProperty(const QString& name = QString(), int default_value = 0,
                         const QString& description = QString(),
                         Property* parent = nullptr,
                         const char* changed_slot = nullptr,
                         QObject* receiver = nullptr, int min_value = INT_MIN,
                         int max_value = INT_MAX);

    /// Set the new value for this property.
    /**
     * Overridden from Property::setValue() to enforce minimum and maximum.
     */
    bool setValue(const QVariant& new_value) override;

    /// Return the internal property value as an integer.
    virtual int getInt() const;

    /// Set the minimum value to be enforced.
    void setMin(int min);

    /// Set the minimum value enforced.
    int getMin();

    /// Set the maximum value to be enforced.
    void setMax(int max);

    /// Get the maximum value to be enforced.
    int getMax();

    /// Called when the editor is created.
    /**
     * Overridden to create a QSpinBox with the min and max set and with a
     * signal/slot connection to setInt(), so the Property value updates every
     * time the value changes, not just when "return" is pressed.
     */
    QWidget* createEditor(QWidget* parent,
                          const QStyleOptionViewItem& option) override;

public Q_SLOTS:
    /// Set the value of this property to the given integer.
    void setInt(int new_value);

private:
    int min_;
    int max_;
};

}  // namespace properties
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__PROPERTIES__INT_PROPERTY_HPP_

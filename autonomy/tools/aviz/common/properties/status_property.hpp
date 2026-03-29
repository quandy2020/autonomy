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

#ifndef AVIZ_COMMON__PROPERTIES__STATUS_PROPERTY_HPP_
#define AVIZ_COMMON__PROPERTIES__STATUS_PROPERTY_HPP_

#include <QIcon>

#include "autonomy/tools/aviz/common/properties/property.hpp"

namespace aviz {
namespace common {
namespace properties {

class StatusProperty : public Property
{
    Q_OBJECT

public:
    enum Level {
        Ok = 0,
        Warn = 1,
        Error = 2
    };  // values index into status_colors_ array.

    StatusProperty(const QString& name, const QString& text, Level level,
                   Property* parent);

    /// Set the status text.
    bool setValue(
        const QVariant& new_value) override;  // Overridden from Property.

    /// Return data appropriate for the given column (0 or 1) and role for this
    /// StatusProperty.
    QVariant getViewData(int column, int role) const override;

    /// Return item flags appropriate for the given column (0 or 1) for this
    /// StatusProperty.
    Qt::ItemFlags getViewFlags(int column) const override;

    /// Return the color appropriate for the given status level.
    static QColor statusColor(Level level);

    /// Return the word appropriate for the given status level: "Ok", "Warn", or
    /// "Error".
    static QString statusWord(Level level);

    /// Get the status icon.
    QIcon statusIcon(Level level) const;

    /// Set the status level.
    virtual void setLevel(Level level);

    /// Get the status level.
    virtual Level getLevel() const;

protected:
    Level level_;

private:
    static QColor status_colors_[3];
    static QString status_words_[3];
    QIcon status_icons_[3];
};

typedef StatusProperty::Level StatusLevel;

}  // namespace properties
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__PROPERTIES__STATUS_PROPERTY_HPP_

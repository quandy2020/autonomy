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

#ifndef AVIZ_COMMON__PROPERTIES__ENUM_PROPERTY_HPP_
#define AVIZ_COMMON__PROPERTIES__ENUM_PROPERTY_HPP_

#include <QStringList>
#include <string>

#include "autonomy/tools/aviz/common/properties/string_property.hpp"

namespace aviz {
namespace common {
namespace properties {

/// Enum property - works like a string property but with integer option values
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
    explicit EnumProperty(const QString& name = QString(),
                          const QString& default_value = QString(),
                          const QString& description = QString(),
                          Property* parent = nullptr,
                          const char* changed_slot = nullptr,
                          QObject* receiver = nullptr);

    /// Clear the list of options.
    /**
     * Does not change the current value of the property.
     */
    virtual void clearOptions();

    virtual void addOption(const QString& option, int value = 0);

    void addOptionStd(const std::string& option, int value = 0);

    /// Get the integer value of the currently selected option.
    int getOptionInt();

    QWidget* createEditor(QWidget* parent,
                          const QStyleOptionViewItem& option) override;

    void sortOptions();

Q_SIGNALS:
    void requestOptions(EnumProperty* property);

private Q_SLOTS:
    void setString(const QString& str);

    void setStringStd(const std::string& str);

private:
    QStringList strings_;
    QHash<QString, int> ints_;
};

}  // namespace properties
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__PROPERTIES__ENUM_PROPERTY_HPP_

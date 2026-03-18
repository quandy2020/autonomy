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

#include <cstdint>
#include <map>
#include <string>

#include <QObject>  // NOLINT: cpplint cannot handle the include order here
#include <QString>  // NOLINT: cpplint cannot handle the include order here

#include "autoview/common/properties/bool_property.hpp"

namespace autoview {
namespace common {

class DisplayContext;

class Display;

namespace properties {

class Property;

class BoolProperty;

class DisplayVisibilityProperty;

/*
 * @brief Changes one visibility bit of a given Display
 */
class DisplayVisibilityProperty : public BoolProperty
{
  Q_OBJECT

public:
  DisplayVisibilityProperty(
    uint32_t vis_bit,
    Display * display,
    const QString & name = QString(),
    bool default_value = false,
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  ~DisplayVisibilityProperty() override;

  virtual void update();

  bool getBool() const override;

  Qt::ItemFlags getViewFlags(int column) const override;

public Q_SLOTS:
  bool setValue(const QVariant & new_value) override;

protected:
  uint32_t vis_bit_;
  Display * display_;
  bool custom_name_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
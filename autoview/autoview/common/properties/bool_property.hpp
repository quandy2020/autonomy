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

// Property specialized to provide getter for booleans.
class BoolProperty : public Property
{
  Q_OBJECT

public:
  explicit BoolProperty(
    const QString& name = QString(),
    bool default_value = false,
    const QString& description = QString(),
    Property* parent = nullptr,
    const char* changed_slot = nullptr,
    QObject* receiver = nullptr);

  ~BoolProperty() override;

  /// Return the value of this.
  virtual bool getBool() const;

  /// If given true, disable this properties own children when this has a value of false.
  void setDisableChildrenIfFalse(bool disable);

  /// Return the disabled children if this has a value of false.
  bool getDisableChildrenIfFalse();

  /// Return the disabled children.
  bool getDisableChildren() override;

public Q_SLOTS:
  /// Set the value of this.
  bool setBool(bool value);

private:
  bool disable_children_if_false_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview

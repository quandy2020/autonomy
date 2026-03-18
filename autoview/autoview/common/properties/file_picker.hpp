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

#include "autoview/common/properties/line_edit_with_button.hpp"
#include "autoview/common/properties/string_property.hpp"

namespace autoview {
namespace common {
namespace properties {

class FilePicker : public LineEditWithButton
{
public:
  explicit FilePicker(
    StringProperty * file_name_property = nullptr,
    QWidget * parent = nullptr);

protected:
  void onButtonClick() override;

private:
  StringProperty * file_name_property_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
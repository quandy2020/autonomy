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

 #include "autoview/common/properties/file_picker.hpp"

#include <QFileDialog>

namespace autoview {
namespace common {
namespace properties {

FilePicker::FilePicker(
  StringProperty * file_name_property,
  QWidget * parent)
: LineEditWithButton(parent),
  file_name_property_(file_name_property)
{}

void FilePicker::onButtonClick()
{
  auto dialog = new QFileDialog(parentWidget());

  connect(
    dialog, SIGNAL(fileSelected(const QString&)),
    file_name_property_, SLOT(setString(const QString&)));

  dialog->exec();
}

}  // namespace properties
}  // namespace common
}  // namespace autoview

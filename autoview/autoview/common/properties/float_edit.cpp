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

#include "autoview/common/properties/float_edit.hpp"

#include <QDoubleValidator>
#include <QLocale>

namespace autoview {
namespace common {
namespace properties {

FloatEdit::FloatEdit(QWidget * parent)
: QLineEdit(parent)
{
  setFrame(false);
  setValidator(new QDoubleValidator(this));
  connect(this, SIGNAL(textEdited(const QString&)), this, SLOT(updateValue()));
}

float FloatEdit::getValue() const
{
  return value_;
}

void FloatEdit::setValue(float new_value)
{
  if (value_ != new_value) {
    QLocale locale;
    value_ = new_value;
    bool ok = true;
    float existing_text_value = locale.toFloat(text(), &ok);
    if (!ok || existing_text_value != new_value) {
      setText(locale.toString(static_cast<double>(value_)));
    }
  }
}

void FloatEdit::updateValue()
{
  if (hasAcceptableInput() ) {
    bool ok = true;
    float new_value = QLocale().toFloat(text(), &ok);
    if (ok) {
      setValue(new_value);
    }
  }
}

}  // namespace properties
}  // namespace common
}  // namespace autoview

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

#include <QLineEdit>

class QPushButton;

namespace autoview {
namespace common {
namespace properties {

/**
 * A QLineEdit with a square button on the right side that says "...".
 */
class LineEditWithButton : public QLineEdit
{
  Q_OBJECT

public:
  explicit LineEditWithButton(QWidget * parent = 0);

  /** Returns the child button.  Use this to connect() something to a
   * button click. */
  QPushButton * button() {return button_;}

protected:
  virtual void resizeEvent(QResizeEvent * event);

protected Q_SLOTS:
  /** @brief Override this to do something when the button is clicked. */
  virtual void onButtonClick() {}

private:
  QPushButton * button_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
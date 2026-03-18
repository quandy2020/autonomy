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

 #include "autoview/common/properties/line_edit_with_button.hpp"

 #include <QPushButton>
 #include <QLineEdit>
 #include <QHBoxLayout>
 #include <QKeyEvent>
 #include <QApplication>
 
 namespace autoview {
 namespace common {
 namespace properties {

 
 LineEditWithButton::LineEditWithButton(QWidget * parent)
 : QLineEdit(parent)
 {
   button_ = new QPushButton(this);
   button_->setText("...");
   button_->setCursor(Qt::ArrowCursor);
   button_->setDefault(false);
   button_->setAutoDefault(false);
   button_->setFocusPolicy(Qt::NoFocus);
 
   connect(button_, SIGNAL(clicked()), this, SLOT(onButtonClick()));
 }
 
 void LineEditWithButton::resizeEvent(QResizeEvent * event)
 {
   // This widget does not use a QLayout object, because I want the
   // child button to be sized and positioned a specific way.  This
   // function lays out the child button based on the new size of the
   // LineEdit.
 
   int padding = 1;
 
   // Make sure the text area doesn't go under the button.
   setTextMargins(padding, padding, height(), padding);
 
   // Call the original resize handler.
   QLineEdit::resizeEvent(event);
 
   // Make the button square, matching the height of this widget minus
   // padding, and located all the way to the right.
   int button_width = height() - 2 * padding;
   int button_height = button_width;
   button_->setGeometry(
     width() - button_width - padding, padding,
     button_width, button_height);
 }
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview

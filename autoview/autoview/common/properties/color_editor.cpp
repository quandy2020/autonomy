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

#include <stdio.h>
#include <QMetaObject>
#include <QMetaProperty>

#include <QPainter>
#include <QColorDialog>

#include "autoview/common/properties/color_property.hpp"
#include "autoview/common/properties/parse_color.hpp"
#include "autoview/common/properties/color_editor.hpp"
 
namespace autoview {
namespace common {
namespace properties {
 
 ColorEditor::ColorEditor(ColorProperty * property, QWidget * parent)
 : LineEditWithButton(parent),
   property_(property)
 {
   connect(
     this, SIGNAL(textChanged(const QString&)),
     this, SLOT(parseText()));
 }
 
 void ColorEditor::paintEvent(QPaintEvent * event)
 {
   LineEditWithButton::paintEvent(event);
   QPainter painter(this);
   painter.setPen(Qt::black);
   paintColorBox(&painter, rect(), color_);
 }
 
 void ColorEditor::paintColorBox(QPainter * painter, const QRect & rect, const QColor & color)
 {
   int padding = 3;
   int size = rect.height() - padding * 2 - 1;
   painter->save();
   painter->setBrush(color);
   painter->drawRoundedRect(
     rect.x() + padding + 3,
     rect.y() + padding, size, size, 0, 0, Qt::AbsoluteSize);
   painter->restore();
 }
 
 void ColorEditor::resizeEvent(QResizeEvent * event)
 {
   // Do the normal line-edit-with-button thing
   LineEditWithButton::resizeEvent(event);
 
   // Then add text padding on the left to make room for the color swatch
   QMargins marge = textMargins();
   setTextMargins(height(), marge.top(), marge.right(), marge.bottom() );
 }
 
 void ColorEditor::parseText()
 {
   QColor new_color = parseColor(text() );
   if (new_color.isValid() ) {
     color_ = new_color;
     if (property_) {
       property_->setColor(new_color);
     }
   }
 }
 
 void ColorEditor::setColor(const QColor & color)
 {
   color_ = color;
   setText(printColor(color));
   if (property_) {
     property_->setColor(color);
   }
 }
 
 void ColorEditor::onButtonClick()
 {
   ColorProperty * prop = property_;
   QColor original_color = prop->getColor();
 
   QColorDialog dialog(color_, parentWidget());
 
   connect(
     &dialog, SIGNAL(currentColorChanged(const QColor&)),
     property_, SLOT(setColor(const QColor&)));
 
   // Without this connection the PropertyTreeWidget does not update
   // the color info "live" when it changes in the dialog and the 3D
   // view.
   connect(
     &dialog, SIGNAL(currentColorChanged(const QColor&)),
     parentWidget(), SLOT(update()));
 
   // On the TWM window manager under linux, and on OSX, this
   // ColorEditor object is destroyed when (or soon after) the dialog
   // opens.  Therefore, here we delete this ColorEditor immediately to
   // force them all to act the same.
   deleteLater();
 
   // dialog->exec() will call an event loop internally, so
   // deleteLater() will take effect and "this" will be destroyed.
   // Therefore, everything we do in this function after dialog->exec()
   // should only use variables on the stack, not member variables.
   if (dialog.exec() != QDialog::Accepted) {
     prop->setColor(original_color);
   }
 }
 
}  // namespace properties
}  // namespace common
}  // namespace autoview

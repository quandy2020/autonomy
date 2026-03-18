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

#include <QPainter>
#include <QStringList>
#include <QStyleOptionViewItem>

#include "autoview/common/properties/parse_color.hpp"
#include "autoview/common/properties/color_property.hpp"
#include "autoview/common/properties/color_editor.hpp"

namespace autoview {
namespace common {
namespace properties {
 
 ColorProperty::ColorProperty(
   const QString & name,
   const QColor & default_value,
   const QString & description,
   Property * parent,
   const char * changed_slot,
   QObject * receiver)
 : Property(name, QVariant(), description, parent, changed_slot, receiver),
   color_(default_value)
 {
   updateString();
 }
 
 bool ColorProperty::setColor(const QColor & new_color)
 {
   if (new_color != color_) {
     Q_EMIT aboutToChange();
     color_ = new_color;
     updateString();
     Q_EMIT changed();
     return true;
   }
   return false;
 }
 
 bool ColorProperty::setValue(const QVariant & new_value)
 {
   if (new_value.type() == QVariant::Color) {
     return setColor(new_value.value<QColor>());
   }
 
   QColor new_color = parseColor(new_value.toString());
   if (new_color.isValid()) {
     return setColor(new_color);
   }
   return false;
 }
 
 void ColorProperty::updateString()
 {
   value_ = printColor(color_);
 }
 
 bool ColorProperty::paint(
   QPainter * painter,
   const QStyleOptionViewItem & option) const
 {
   painter->save();
   QColor color = color_;
   if (!(getViewFlags(0) & Qt::ItemIsEnabled)) {
     color = QColor(200, 200, 200);
     painter->setPen(QColor(Qt::lightGray));
   }
   QString text = value_.toString();
   QRect rect = option.rect;
   ColorEditor::paintColorBox(painter, rect, color);
   rect.adjust(rect.height() + 4, 1, 0, 0);
   painter->drawText(rect, text);
 
   painter->restore();
 
   return true;  // return true, since this function has done the painting.
 }
 
 QWidget * ColorProperty::createEditor(QWidget * parent, const QStyleOptionViewItem & option)
 {
   Q_UNUSED(option);
   ColorEditor * editor = new ColorEditor(this, parent);
   editor->setFrame(false);
   return editor;
 }
 
 QColor ColorProperty::getColor() const
 {
   return color_;
 }
 
 Ogre::ColourValue ColorProperty::getOgreColor() const
 {
   return qtToOgre(color_);
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
 
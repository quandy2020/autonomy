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

 #include "autoview/common/properties/int_property.hpp"

 #include <climits>  
 
 #include <QtGlobal>  
 #include <QSpinBox>  
 
 namespace autoview {
 namespace common {
 namespace properties {
 
 IntProperty::IntProperty(
   const QString & name,
   int default_value,
   const QString & description,
   Property * parent,
   const char * changed_slot,
   QObject * receiver,
   int min_value,
   int max_value)
 : Property(name, default_value, description, parent, changed_slot, receiver),
   min_(min_value),
   max_(max_value)
 {
 }
 
 bool IntProperty::setValue(const QVariant & new_value)
 {
   return Property::setValue(qBound(min_, new_value.toInt(), max_));
 }
 
 int IntProperty::getInt() const
 {
   return getValue().toInt();
 }
 
 void IntProperty::setMin(int min)
 {
   min_ = min;
   setValue(getValue() );
 }
 
 int IntProperty::getMin()
 {
   return min_;
 }
 
 void IntProperty::setMax(int max)
 {
   max_ = max;
   setValue(getValue() );
 }
 
 int IntProperty::getMax()
 {
   return max_;
 }
 
 QWidget * IntProperty::createEditor(QWidget * parent, const QStyleOptionViewItem & option)
 {
   Q_UNUSED(option);
   QSpinBox * editor = new QSpinBox(parent);
   editor->setFrame(false);
   editor->setRange(min_, max_);
   connect(editor, SIGNAL(valueChanged(int)), this, SLOT(setInt(int)));
   return editor;
 }
 
 void IntProperty::setInt(int new_value)
 {
   setValue(new_value);
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
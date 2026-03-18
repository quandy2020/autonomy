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

 #include "autoview/common/properties/float_property.hpp"

 #include <cfloat>  
 
 #include <QtGlobal> 
 
 namespace autoview {
 namespace common {
 namespace properties {
 
 FloatProperty::FloatProperty(
   const QString & name,
   float default_value,
   const QString & description,
   Property * parent,
   const char * changed_slot,
   QObject * receiver)
 : Property(name, default_value, description, parent, changed_slot, receiver),
   min_(-FLT_MAX),
   max_(FLT_MAX)
 {
 }
 
 bool FloatProperty::setValue(const QVariant & new_value)
 {
   return Property::setValue(qBound(min_, new_value.toFloat(), max_));
 }
 
 float FloatProperty::getFloat() const
 {
   return getValue().toFloat();
 }
 
 void FloatProperty::setMin(float min)
 {
   min_ = min;
   setValue(getValue());
 }
 
 float FloatProperty::getMin()
 {
   return min_;
 }
 
 void FloatProperty::setMax(float max)
 {
   max_ = max;
   setValue(getValue());
 }
 
 float FloatProperty::getMax()
 {
   return max_;
 }
 
 bool FloatProperty::setFloat(float new_value)
 {
   return setValue(new_value);
 }
 
 bool FloatProperty::add(float delta)
 {
   return setValue(delta + getValue().toFloat());
 }
 
 bool FloatProperty::multiply(float factor)
 {
   return setValue(factor * getValue().toFloat());
 }
 
 }  // end namespace properties
 }  // end namespace common
 }  // end namespace autoview
 
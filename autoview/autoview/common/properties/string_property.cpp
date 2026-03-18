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

 #include <string>

 #include "autoview/common/properties/string_property.hpp"
 
 namespace autoview {
 namespace common {
 namespace properties {
 
 StringProperty::StringProperty(
   const QString & name,
   const QString & default_value,
   const QString & description,
   Property * parent,
   const char * changed_slot,
   QObject * receiver)
 : Property(name, default_value, description, parent, changed_slot, receiver)
 {}
 
 std::string StringProperty::getStdString() const
 {
   return getValue().toString().toStdString();
 }
 
 QString StringProperty::getString() const
 {
   return getValue().toString();
 }
 
 bool StringProperty::setStdString(const std::string & std_str)
 {
   return setValue(QString::fromStdString(std_str));
 }
 
 bool StringProperty::setString(const QString & str)
 {
   return setValue(str);
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
 
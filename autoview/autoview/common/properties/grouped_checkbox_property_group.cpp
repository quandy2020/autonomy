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

 #include "autoview/common/properties/grouped_checkbox_property.hpp"
 #include "autoview/common/properties/grouped_checkbox_property_group.hpp"
 
 namespace autoview {
 namespace common {
 namespace properties {
 
 void GroupedCheckboxPropertyGroup::addProperty(GroupedCheckboxProperty * property)
 {
   properties_.push_back(property);
 }
 
 void GroupedCheckboxPropertyGroup::setChecked(GroupedCheckboxProperty * property_to_check)
 {
   for (const auto & property : properties_) {
     if (property) {
       property->setRawValue(property == property_to_check);
     }
   }
 }
 
 GroupedCheckboxProperty * GroupedCheckboxPropertyGroup::getChecked()
 {
   for (const auto & property : properties_) {
     if (property && property->getValue().toBool()) {
       return property;
     }
   }
   return nullptr;
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
 
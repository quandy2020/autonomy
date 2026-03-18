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

 #include "autoview/common/properties/enum_property.hpp"

 #include <string>
 
 #include "autoview/common/properties/combo_box.hpp"
 
 namespace autoview {
 namespace common {
 namespace properties {
 
 EnumProperty::EnumProperty(
   const QString & name,
   const QString & default_value,
   const QString & description,
   Property * parent,
   const char * changed_slot,
   QObject * receiver)
 : StringProperty(name, default_value, description, parent, changed_slot, receiver)
 {
 }
 
 void EnumProperty::clearOptions()
 {
   strings_.clear();
   ints_.clear();
 }
 
 void EnumProperty::addOption(const QString & option, int value)
 {
   strings_.push_back(option);
   ints_[option] = value;
 }
 
 int EnumProperty::getOptionInt()
 {
   QString current_string = getValue().toString();
   QHash<QString, int>::const_iterator int_iter = ints_.find(current_string);
   if (int_iter != ints_.end() ) {
     return int_iter.value();
   }
   return 0;
 }
 
 void EnumProperty::addOptionStd(const std::string & option, int value)
 {
   addOption(QString::fromStdString(option), value);
 }
 
 QWidget * EnumProperty::createEditor(QWidget * parent, const QStyleOptionViewItem & option)
 {
   Q_UNUSED(option);
   // Emit requestOptions() to give listeners a chance to change the option list.
   Q_EMIT requestOptions(this);
 
   ComboBox * cb = new ComboBox(parent);
   cb->addItems(strings_);
   cb->setCurrentIndex(strings_.indexOf(getValue().toString() ));
   QObject::connect(
     cb, SIGNAL(currentIndexChanged(const QString&)), this,
     SLOT(setString(const QString&)));
 
   // TODO(anyone): need to better handle string value which is not in list.
   return cb;
 }
 
 void EnumProperty::setString(const QString & str)
 {
   setValue(str);
 }
 
 void EnumProperty::setStringStd(const std::string & str)
 {
   setString(QString::fromStdString(str));
 }
 
 void EnumProperty::sortOptions()
 {
   strings_.sort();
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
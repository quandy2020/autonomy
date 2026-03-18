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

 #include <QApplication>
 #include <QColor>
 #include <QPalette>
 
 #include "autoview/common/load_resource.hpp"
 #include "autoview/common/properties/property_tree_model.hpp"
 #include "autoview/common/properties/status_property.hpp"
 
 namespace autoview {
 namespace common {
 namespace properties {

 QColor StatusProperty::status_colors_[3] = {QColor(), QColor(192, 128, 0), QColor(192, 32, 32)};
 QString StatusProperty::status_words_[3] = {"Ok", "Warn", "Error"};
 
 StatusProperty::StatusProperty(
   const QString & name,
   const QString & text,
   StatusProperty::Level level,
   Property * parent)
 : Property(name, text, text, parent),
   level_(level)
 {
   setShouldBeSaved(false);
   status_icons_[0] = loadPixmap("package://rviz_common/icons/ok.png");
   status_icons_[1] = loadPixmap("package://rviz_common/icons/warning.png");
   status_icons_[2] = loadPixmap("package://rviz_common/icons/error.png");
 
   if (!status_colors_[0].isValid()) {  // initialize default text color once
     status_colors_[0] = QApplication::palette().color(QPalette::Text);
   }
 }
 
 bool StatusProperty::setValue(const QVariant & new_value)
 {
   setDescription(new_value.toString() );
   return Property::setValue(new_value);
 }
 
 QVariant StatusProperty::getViewData(int column, int role) const
 {
   if ( (getViewFlags(column) & Qt::ItemIsEnabled) && column == 0 && role == Qt::ForegroundRole) {
     return statusColor(level_);
   }
   if (column == 0 && role == Qt::DecorationRole) {
     return statusIcon(level_);
   }
   return Property::getViewData(column, role);
 }
 
 Qt::ItemFlags StatusProperty::getViewFlags(int column) const
 {
   return Property::getViewFlags(column);
 }
 
 // static function
 QColor StatusProperty::statusColor(StatusProperty::Level level)
 {
   return status_colors_[static_cast<int>(level)];
 }
 
 // static function
 QIcon StatusProperty::statusIcon(StatusProperty::Level level) const
 {
   return status_icons_[level];
 }
 
 // static function
 QString StatusProperty::statusWord(StatusProperty::Level level)
 {
   return status_words_[static_cast<int>(level)];
 }
 
 
 void StatusProperty::setLevel(StatusProperty::Level level)
 {
   if (level_ != level) {
     level_ = level;
     if (model_) {
       model_->emitDataChanged(this);
     }
   }
 }
 
 StatusProperty::Level StatusProperty::getLevel() const
 {
   return level_;
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
 
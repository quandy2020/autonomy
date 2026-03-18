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

 #include "autoview/common/properties/property_tree_delegate.hpp"

 #include <QAbstractItemView>  
 
 #include "autoview/common/properties/line_edit_with_button.hpp"
 #include "autoview/common/properties/property.hpp"
 
 namespace autoview {
 namespace common {
 namespace properties {

 
 PropertyTreeDelegate::PropertyTreeDelegate(QObject * parent_object)
 : QStyledItemDelegate(parent_object)
 {
 }
 
 void PropertyTreeDelegate::paint(
   QPainter * painter,
   const QStyleOptionViewItem & option,
   const QModelIndex & index) const
 {
   Property * prop = static_cast<Property *>( index.internalPointer() );
   if (!prop || !prop->paint(painter, option)) {
     QStyledItemDelegate::paint(painter, option, index);
   }
 }
 
 QWidget * PropertyTreeDelegate::createEditor(
   QWidget * parent,
   const QStyleOptionViewItem & option,
   const QModelIndex & index) const
 {
   Property * prop = static_cast<Property *>( index.internalPointer() );
   if (!prop || prop->getReadOnly()) {
     return 0;
   }
 
   if (QWidget * editor = prop->createEditor(parent, option)) {
     return editor;
   }
   return QStyledItemDelegate::createEditor(parent, option, index);
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview

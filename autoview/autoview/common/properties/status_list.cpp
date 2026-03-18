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

 #include "autoview/common/properties/status_list.hpp"

 #include <cstdio>
 
 #include "autoview/common/properties/property_tree_model.hpp"
 
 namespace autoview {
 namespace common {
 namespace properties {

 
 StatusList::StatusList(const QString & name, Property * parent)
 : StatusProperty("", "", Ok, parent)
 {
   setName(name);
   setShouldBeSaved(false);
 }
 
 void StatusList::setName(const QString & name)
 {
   name_prefix_ = name;
   updateLabel();
 }
 
 void StatusList::setStatus(Level level, const QString & name, const QString & text)
 {
   QHash<QString, StatusProperty *>::iterator child_iter = status_children_.find(name);
   StatusProperty * child;
   if (child_iter == status_children_.end()) {
     child = new StatusProperty(name, text, level, this);
     child->setReadOnly(true);
     status_children_.insert(name, child);
   } else {
     child = child_iter.value();
     child->setLevel(level);
     child->setValue(text);
   }
   if (level > level_) {
     setLevel(level);
   } else if (level < level_) {
     updateLevel();
   }
 }
 
 void StatusList::deleteStatus(const QString & name)
 {
   StatusProperty * child = status_children_.take(name);
   if (child) {
     delete child;
     updateLevel();
   }
 }
 
 void StatusList::clear()
 {
   int num_rows = numChildren();
   if (num_rows > 0) {
     QList<StatusProperty *> to_be_deleted = status_children_.values();
 
     status_children_.clear();
 
     // It is important here to clear the status_children_ list before
     // deleting its contents.  On Macs the deletion can indirectly
     // trigger a call to setStatus(), and status_children_ should not
     // contain any pointers to deleted memory at that time.
     for (int i = 0; i < to_be_deleted.size(); i++) {
       delete to_be_deleted[i];
     }
   }
   setLevel(Ok);
 }
 
 void StatusList::updateLevel()
 {
   Level new_level = Ok;
 
   QHash<QString, StatusProperty *>::iterator iter;
   for (iter = status_children_.begin(); iter != status_children_.end(); iter++) {
     Level child_level = iter.value()->getLevel();
     if (child_level > new_level) {
       new_level = child_level;
     }
   }
   setLevel(new_level);
 }
 
 void StatusList::setLevel(Level new_level)
 {
   StatusProperty::setLevel(new_level);
   updateLabel();
 }
 
 void StatusList::updateLabel()
 {
   StatusProperty::setName(name_prefix_ + ": " + statusWord(getLevel()));
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
 
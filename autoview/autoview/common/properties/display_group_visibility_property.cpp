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

 #include <map>

 #include "autoview/common/properties/display_group_visibility_property.hpp"
 
 #include "autoview/common/properties/bool_property.hpp"
 #include "autoview/common/display_context.hpp"
 #include "autoview/common/bit_allocator.hpp"
 #include "autoview/common/display.hpp"
 #include "autoview/common/display_group.hpp"
 
 namespace autoview {
 namespace common {
 namespace properties {
 
 DisplayGroupVisibilityProperty::DisplayGroupVisibilityProperty(
   uint32_t vis_bit,
   DisplayGroup * display_group,
   Display * parent_display,
   const QString & name,
   bool default_value,
   const QString & description,
   Property * parent,
   const char * changed_slot,
   QObject * receiver)
 : DisplayVisibilityProperty(
     vis_bit, display_group, name, default_value, description, parent, changed_slot, receiver),
   display_group_(display_group),
   parent_display_(parent_display)
 {
   connect(
     display_group, SIGNAL(displayAdded(Display*)),
     this, SLOT(onDisplayAdded(Display*)));
   connect(
     display_group, SIGNAL(displayRemoved(Display*)),
     this, SLOT(onDisplayRemoved(Display*)));
 
   for (int i = 0; i < display_group->numDisplays(); i++) {
     rviz_common::Display * display = display_group->getDisplayAt(i);
     if (display != parent_display) {
       onDisplayAdded(display);
     }
   }
 
   setDisableChildrenIfFalse(true);
 }
 
 void DisplayGroupVisibilityProperty::update()
 {
   DisplayVisibilityProperty::update();
   for (auto const & it : disp_vis_props_) {
     it.second->update();
   }
 }
 
 void DisplayGroupVisibilityProperty::sortDisplayList()
 {
   // remove and re-add everything in our property list
   // in the same order as it appears in the display group
   for (int i = 0; i < display_group_->numDisplays(); i++) {
     rviz_common::Display * display = display_group_->getDisplayAt(i);
     auto it = disp_vis_props_.find(display);
     if (it != disp_vis_props_.end()) {
       takeChild(it->second);
       addChild(it->second);
     }
   }
 }
 
 void DisplayGroupVisibilityProperty::onDisplayAdded(Display * display)
 {
   auto display_group = qobject_cast<DisplayGroup *>(display);
   DisplayVisibilityProperty * vis_prop;
   if (display_group) {
     vis_prop = new DisplayGroupVisibilityProperty(
       vis_bit_, display_group, parent_display_, "",
       true, "Uncheck to hide everything in this Display Group", this);
   } else {
     vis_prop = new DisplayVisibilityProperty(
       vis_bit_, display, "", true,
       "Show or hide this Display", this);
   }
   disp_vis_props_[display] = vis_prop;
   sortDisplayList();
 }
 
 void DisplayGroupVisibilityProperty::onDisplayRemoved(Display * display)
 {
   auto it = disp_vis_props_.find(display);
   if (it != disp_vis_props_.end()) {
     Property * child = takeChild(it->second);
     child->setParent(nullptr);
     delete child;
     disp_vis_props_.erase(display);
   }
 }
 
 
 DisplayGroupVisibilityProperty::~DisplayGroupVisibilityProperty() = default;
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
 
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
 #include <string>
 #include <vector>
 
 #include <QApplication>  // NOLINT: cpplint can't handle Qt imports
 
 #include "autoview/common/properties/ros_topic_property.hpp"
//  #include "autoview/common/ros_integration/ros_node_abstraction_iface.hpp"
 #include "autoview/common/properties/topic_property.hpp"
 
 namespace autoview {
 namespace common {
 namespace properties {
 
 TopicProperty::TopicProperty(
   const QString & name,
   const QString & default_value,
   const QString & message_type,
   const QString & description,
   Property * parent,
   const char * changed_slot,
   QObject * receiver)
 : EditableEnumProperty(name, default_value, description, parent, changed_slot, receiver),
   rviz_ros_node_(),
   message_type_(message_type)
 {
   connect(
     this, SIGNAL(requestOptions(EditableEnumProperty*)),
     this, SLOT(fillTopicList()));
 }
 
 void TopicProperty::initialize(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
 {
   rviz_ros_node_ = rviz_ros_node;
 }
 
 void TopicProperty::setMessageType(const QString & message_type)
 {
   message_type_ = message_type;
 }
 
 void TopicProperty::fillTopicList()
 {
   QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
   clearOptions();
 
   std::string std_message_type = message_type_.toStdString();
   std::map<std::string, std::vector<std::string>> published_topics =
     rviz_ros_node_.lock()->get_topic_names_and_types();
 
   for (const auto & topic : published_topics) {
     // Only add topics whose type matches.
     for (const auto & type : topic.second) {
       if (type == std_message_type) {
         addOptionStd(topic.first);
       }
     }
   }
   sortOptions();
   QApplication::restoreOverrideCursor();
 }
 
 FilteredTopicProperty::FilteredTopicProperty(
   const QString & name,
   const QString & default_value,
   const QString & message_type,
   const QString & description,
   const QRegExp & filter,
   Property * parent,
   const char * changed_slot,
   QObject * receiver)
 : RosTopicProperty(name, default_value, message_type, description, parent, changed_slot, receiver)
   , filter_(filter)
   , filter_enabled_(true)
 {
 }
 
 void FilteredTopicProperty::enableFilter(bool enabled)
 {
   filter_enabled_ = enabled;
   fillTopicList();
 }
 
 QRegExp FilteredTopicProperty::filter() const
 {
   return filter_;
 }
 
 void FilteredTopicProperty::fillTopicList()
 {
   QStringList filtered_strings_;
 
   // Obtain list of available topics
   RosTopicProperty::fillTopicList();
   // Apply filter
   if (filter_enabled_) {
     strings_ = strings_.filter(filter_);
   }
 }
 
}  // end namespace properties
}  // end namespace common
}  // namespace autoview

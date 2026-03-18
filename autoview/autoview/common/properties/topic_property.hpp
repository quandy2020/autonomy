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

#pragma once

#include <string>

#include "autoview/common/properties/editable_enum_property.hpp"
#include "autoview/common/integration/node_abstraction_iface.hpp"

namespace autoview {
namespace common {
namespace properties {

class RosTopicProperty : public EditableEnumProperty
{
  Q_OBJECT

public:
  explicit RosTopicProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & message_type = QString(),
    const QString & description = QString(),
    Property * parent = nullptr,
    const char * changed_slot = nullptr,
    QObject * receiver = nullptr);

  void initialize(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node);

  void setMessageType(const QString & message_type);

  QString getMessageType() const
  {return message_type_;}

  QString getTopic() const
  {return getValue().toString();}

  std::string getTopicStd() const
  {return getValue().toString().toStdString();}

  bool isEmpty() const
  {return getTopicStd().empty();}

protected Q_SLOTS:
  virtual void fillTopicList();

private:
  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  QString message_type_;
};

class RVIZ_COMMON_PUBLIC RosFilteredTopicProperty
  : public rviz_common::properties::RosTopicProperty
{
  Q_OBJECT

public:
  RosFilteredTopicProperty(
    const QString & name = QString(),
    const QString & default_value = QString(),
    const QString & message_type = QString(),
    const QString & description = QString(),
    const QRegExp & filter = QRegExp(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  void enableFilter(bool enabled);

  QRegExp filter() const;

protected Q_SLOTS:
  void fillTopicList() override;

private:
  QRegExp filter_;
  bool filter_enabled_;
};

}  // end namespace properties
}  // end namespace common
}  // end namespace autoview
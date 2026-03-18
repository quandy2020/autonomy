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

#include <OgreColourValue.h>

#include <QColor>
#include <QString>

#include "rviz_common/properties/property.hpp"
namespace autoview {
namespace common {
namespace properties {

class ColorProperty : public Property
{
  Q_OBJECT

public:
  ColorProperty(
    const QString & name = QString(),
    const QColor & default_value = Qt::black,
    const QString & description = QString(),
    Property * parent = 0,
    const char * changed_slot = 0,
    QObject * receiver = 0);

  bool setValue(const QVariant & new_value) override;

  bool paint(QPainter * painter, const QStyleOptionViewItem & option) const override;

  QWidget * createEditor(QWidget * parent, const QStyleOptionViewItem & option) override;

  virtual QColor getColor() const;

  Ogre::ColourValue getOgreColor() const;

public Q_SLOTS:
  virtual bool setColor(const QColor & color);

private:
  void updateString();

  QColor color_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
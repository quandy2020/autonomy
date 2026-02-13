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

#ifndef AVIZ_COMMON__PROPERTIES__PARSE_COLOR_HPP_
#define AVIZ_COMMON__PROPERTIES__PARSE_COLOR_HPP_

#include <QColor>
#include <QString>

#include <OgreColourValue.h>

namespace aviz {
namespace common {
namespace properties {

/// Parse a color string to QColor
QColor parseColor(const QString& color_string);

/// Print a QColor to string
QString printColor(const QColor& color);

/// Convert Ogre::ColourValue to QColor
QColor ogreToQt(const Ogre::ColourValue& ogre_color);

/// Convert QColor to Ogre::ColourValue
Ogre::ColourValue qtToOgre(const QColor& qt_color);

}  // namespace properties
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__PROPERTIES__PARSE_COLOR_HPP_

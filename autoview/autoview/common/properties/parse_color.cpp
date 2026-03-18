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

 #include "autoview/common/properties/parse_color.hpp"

 namespace autoview {
 namespace common {
 namespace properties {
 
 static int limit(int i)
 {
   if (i < 0) {
     return 0;
   }
   if (i > 255) {
     return 255;
   }
   return i;
 }
 
 QColor parseColor(const QString & color_string)
 {
   if (color_string.indexOf(';') != -1) {
     QStringList strings = color_string.split(';');
     if (strings.size() >= 3) {
       bool r_ok = true;
       int r = strings[0].toInt(&r_ok);
       bool g_ok = true;
       int g = strings[1].toInt(&g_ok);
       bool b_ok = true;
       int b = strings[2].toInt(&b_ok);
       if (r_ok && g_ok && b_ok) {
         return QColor(limit(r), limit(g), limit(b));
       }
     }
     return QColor();
   }
 
   QColor new_color;
   if (QColor::colorNames().contains(color_string, Qt::CaseInsensitive) ||
     (color_string.size() > 0 && color_string[0] == '#' ))
   {
     new_color.setNamedColor(color_string.toLower() );
   }
   return new_color;
 }
 
 QString printColor(const QColor & color)
 {
   return QString("%1; %2; %3")
          .arg(color.red() )
          .arg(color.green() )
          .arg(color.blue() );
 }
 
 QColor ogreToQt(const Ogre::ColourValue & c)
 {
   return QColor::fromRgbF(c.r, c.g, c.b, c.a);
 }
 
 Ogre::ColourValue qtToOgre(const QColor & c)
 {
   return Ogre::ColourValue(c.redF(), c.greenF(), c.blueF(), c.alphaF() );
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
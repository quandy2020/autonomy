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
 
 #include "autoview/common/properties/line_edit_with_button.hpp"
 
 namespace autoview {
 namespace common {
 namespace properties {
 
 class ColorProperty;
 
 class ColorEditor : public LineEditWithButton
 {
   Q_OBJECT
 
 public:
   explicit ColorEditor(ColorProperty * property = 0, QWidget * parent = 0);
 
   /** Static function to paint just the color box.  Paints it in the
    * left end of rect, size rect.height() by rect.height(). */
   static void paintColorBox(QPainter * painter, const QRect & rect, const QColor & color);
 
 public Q_SLOTS:
   void setColor(const QColor & color);
   void parseText();
 
 protected:
   /** Call parent version then paint color swatch. */
   virtual void paintEvent(QPaintEvent * event);
 
   virtual void resizeEvent(QResizeEvent * event);
 
 protected Q_SLOTS:
   virtual void onButtonClick();
 
private:
   QColor color_;
   ColorProperty * property_;
};
 
}  // namespace properties
}  // namespace common
}  // namespace autoview
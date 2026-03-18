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

 #include "autoview/common/properties/splitter_handle.hpp"

 #include <QEvent>
 #include <QMouseEvent>
 #include <QPainter>
 #include <QPaintEvent>
 #include <QTreeView>
 
 namespace autoview {
 namespace common {
 namespace properties {

 SplitterHandle::SplitterHandle(QTreeView * parent)
 : QWidget(parent),
   parent_(parent),
   first_column_size_ratio_(0.5f),
   color_(128, 128, 128, 64)
 {
   setCursor(Qt::SplitHCursor);
   updateGeometry();
   parent_->installEventFilter(this);
 }
 
 bool SplitterHandle::eventFilter(QObject * event_target, QEvent * event)
 {
   if (event_target == parent_ && (event->type() == QEvent::Resize ||
     event->type() == QEvent::Paint))
   {
     updateGeometry();
   }
   return false;  // Return false regardless so resize event also does its normal job.
 }
 
 void SplitterHandle::updateGeometry()
 {
   int w = 7;
   int content_width = parent_->contentsRect().width();
   int new_column_width = static_cast<int>(first_column_size_ratio_ * content_width);
   if (new_column_width != parent_->columnWidth(0) ) {
     parent_->setColumnWidth(0, new_column_width);
     parent_->setColumnWidth(1, content_width - new_column_width);
   }
   int new_x = new_column_width - w / 2 + parent_->columnViewportPosition(0);
   if (new_x != x() || parent_->height() != height() ) {
     setGeometry(new_x, 0, w, parent_->height() );
   }
 }
 
 void SplitterHandle::setRatio(float ratio)
 {
   first_column_size_ratio_ = ratio;
   updateGeometry();
 }
 
 float SplitterHandle::getRatio()
 {
   return first_column_size_ratio_;
 }
 
 void SplitterHandle::mousePressEvent(QMouseEvent * event)
 {
   if (event->button() == Qt::LeftButton) {
     // position of mouse press inside this QWidget
     x_press_offset_ = event->x();
   }
 }
 
 void SplitterHandle::mouseMoveEvent(QMouseEvent * event)
 {
   int padding = 55;
 
   if (event->buttons() & Qt::LeftButton) {
     QPoint pos_rel_parent = parent_->mapFromGlobal(event->globalPos() );
 
     int new_x = pos_rel_parent.x() - x_press_offset_ - parent_->columnViewportPosition(0);
 
     if (new_x > parent_->width() - width() - padding) {
       new_x = parent_->width() - width() - padding;
     }
 
     if (new_x < padding) {
       new_x = padding;
     }
 
     if (new_x != x() ) {
       int new_column_width = new_x + width() / 2 - parent_->contentsRect().x();
       first_column_size_ratio_ = new_column_width /
         static_cast<float>(parent_->contentsRect().width());
       updateGeometry();
     }
   }
 }
 
 void SplitterHandle::paintEvent(QPaintEvent * event)
 {
   Q_UNUSED(event);
   QPainter painter(this);
   painter.setPen(color_);
   painter.drawLine(1 + width() / 2, 0, 1 + width() / 2, height() );
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
 
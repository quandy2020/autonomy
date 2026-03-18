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

 #include "autoview/common/properties/property_tree_widget.hpp"

 #include <QHash>
 #include <QSet>
 #include <QTimer>
 
 #include "autoview/common/properties/property.hpp"
 #include "autoview/common/properties/property_tree_delegate.hpp"
 #include "autoview/common/properties/splitter_handle.hpp"
 #include "autoview/common/properties/status_list.hpp"
 
 namespace autoview {
 namespace common {
 namespace properties {
 
 PropertyTreeWidget::PropertyTreeWidget(QWidget * parent)
 : QTreeView(parent),
   model_(nullptr),
   splitter_handle_(new SplitterHandle(this))
 {
   setItemDelegateForColumn(1, new PropertyTreeDelegate(this));
   setDropIndicatorShown(true);
   setUniformRowHeights(true);
   setHeaderHidden(true);
   setDragEnabled(true);
   setAcceptDrops(true);
   setAnimated(true);
   setSelectionMode(QAbstractItemView::ExtendedSelection);
   setEditTriggers(QAbstractItemView::AllEditTriggers);
   setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
 
   auto timer = new QTimer(this);
   connect(timer, SIGNAL(timeout()), this, SLOT(update()));
   timer->start(100);
 }
 
 void PropertyTreeWidget::currentChanged(
   const QModelIndex & new_current_index,
   const QModelIndex & previous_current_index)
 {
   QTreeView::currentChanged(new_current_index, previous_current_index);
   Q_EMIT currentPropertyChanged(
     static_cast<const Property *>(
       new_current_index.internalPointer() ));
 }
 
 void PropertyTreeWidget::selectionChanged(
   const QItemSelection & selected,
   const QItemSelection & deselected)
 {
   QTreeView::selectionChanged(selected, deselected);
   Q_EMIT selectionHasChanged();
 }
 
 void PropertyTreeWidget::setModel(PropertyTreeModel * model)
 {
   if (model_) {
     disconnect(
       model_, SIGNAL(propertyHiddenChanged(const Property*)),
       this, SLOT(propertyHiddenChanged(const Property*)));
     disconnect(
       model_, SIGNAL(expand(const QModelIndex&)),
       this, SLOT(expand(const QModelIndex&)));
     disconnect(
       model_, SIGNAL(collapse(const QModelIndex&)),
       this, SLOT(collapse(const QModelIndex&)));
   }
   model_ = model;
   QTreeView::setModel(model_);
   if (model_) {
     connect(
       model_, SIGNAL(propertyHiddenChanged(const Property*)),
       this, SLOT(propertyHiddenChanged(const Property*)), Qt::QueuedConnection);
     connect(
       model_, SIGNAL(expand(const QModelIndex&)),
       this, SLOT(expand(const QModelIndex&)));
     connect(
       model_, SIGNAL(collapse(const QModelIndex&)),
       this, SLOT(collapse(const QModelIndex&)));
 
     // this will trigger all hiddenChanged events to get re-fired
     model_->getRoot()->setModel(model_->getRoot()->getModel() );
   }
 }
 
 void PropertyTreeWidget::propertyHiddenChanged(const Property * property)
 {
   if (model_) {
     setRowHidden(
       property->rowNumberInParent(), model_->parentIndex(property),
       property->getHidden() );
   }
 }
 
 void PropertyTreeWidget::save(Config config) const
 {
   saveExpandedEntries(config.mapMakeChild("Expanded"), QModelIndex(), "");
 
   config.mapSetValue("Splitter Ratio", splitter_handle_->getRatio() );
 }
 
 void PropertyTreeWidget::saveExpandedEntries(
   Config config, const QModelIndex & parent_index,
   const QString & prefix) const
 {
   int num_children = model_->rowCount(parent_index);
   if (num_children > 0) {
     QHash<QString, int> name_counts;
     for (int i = 0; i < num_children; i++) {
       QModelIndex child_index = model_->index(i, 0, parent_index);
       Property * child = model_->getProp(child_index);
       QString child_name = child->getName();
       if (qobject_cast<StatusList *>(child)) {
         // StatusList objects change their name dynamically, so
         // normalize to a standard string.
         child_name = "Status";
       }
       int name_occurrence = ++( name_counts[child_name]);
       QString full_name = prefix + "/" + child_name + QString::number(name_occurrence);
       if (isExpanded(child_index)) {
         config.listAppendNew().setValue(full_name);
       }
       saveExpandedEntries(config, child_index, full_name);
     }
   }
 }
 
 void PropertyTreeWidget::load(const Config & config)
 {
   Config expanded_list_config = config.mapGetChild("Expanded");
   QSet<QString> expanded_full_names;
   int num_expanded = expanded_list_config.listLength();
   for (int i = 0; i < num_expanded; i++) {
     expanded_full_names.insert(expanded_list_config.listChildAt(i).getValue().toString() );
   }
   expandEntries(expanded_full_names, QModelIndex(), "");
 
   float ratio;
   if (config.mapGetFloat("Splitter Ratio", &ratio)) {
     splitter_handle_->setRatio(ratio);
   }
 }
 
 void PropertyTreeWidget::expandEntries(
   const QSet<QString> & expanded_full_names,
   const QModelIndex & parent_index,
   const QString & prefix)
 {
   int num_children = model_->rowCount(parent_index);
   if (num_children > 0) {
     QHash<QString, int> name_counts;
     for (int i = 0; i < num_children; i++) {
       QModelIndex child_index = model_->index(i, 0, parent_index);
       Property * child = model_->getProp(child_index);
       QString child_name = child->getName();
       if (qobject_cast<StatusList *>(child)) {
         child_name = "Status";
       }
       int name_occurrence = ++( name_counts[child_name]);
       QString full_name = prefix + "/" + child_name + QString::number(name_occurrence);
       if (expanded_full_names.contains(full_name)) {
         setExpanded(child_index, true);
       }
       expandEntries(expanded_full_names, child_index, full_name);
     }
   }
 }
 
 }  // namespace properties
 }  // namespace common
 }  // namespace autoview
 
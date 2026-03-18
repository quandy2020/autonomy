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

#include <QTreeView>

#include "autoview/common/properties/property_tree_model.hpp"

namespace autoview {
namespace common {
namespace properties {

class Property;
class SplitterHandle;

class PropertyTreeWidget : public QTreeView
{
  Q_OBJECT

public:
  explicit PropertyTreeWidget(QWidget * parent = 0);

// disable the overloaded virtual warning, since this is intentional
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif
  /// Set the data model this widget should view.
  void setModel(PropertyTreeModel * model);
#ifdef __clang__
# pragma clang diagnostic pop
#endif

  PropertyTreeModel * getModel() const {return model_;}

  /// Return the list of objects of a given type which are currently selected.
  template<class Type>
  QList<Type *> getSelectedObjects()
  {
    QModelIndexList indexes = selectedIndexes();
    int num_selected = indexes.size();

    QList<Type *> objects_out;

    for (int i = 0; i < num_selected; i++) {
      if (indexes[i].column() == 0) {
        Property * prop = model_->getProp(indexes[i]);
        if (prop != model_->getRoot() ) {
          // TODO(greimela) Figure out why qobject_cast does not work here
          auto obj = dynamic_cast<Type *>(prop);
          if (obj) {
            objects_out.push_back(obj);
          }
        }
      }
    }
    return objects_out;
  }

  /// Write state to the given Config.
  void save(Config config) const;

  /// Read state from the given Config.
  void load(const Config & config);

protected:
  /// Called whenever current item changes.
  /**
   * Calls QTreeView implementation then emits currentPropertyChanged().
   */
  void currentChanged(const QModelIndex & current, const QModelIndex & previous) override;

  /// Called whenever selection changes.
  /**
   * Calls QTreeView implementation then emits selectionHasChanged().
   */
  void selectionChanged(
    const QItemSelection & selected, const QItemSelection & deselected) override;

protected Q_SLOTS:
  virtual void propertyHiddenChanged(const Property * property);

Q_SIGNALS:
  void currentPropertyChanged(const Property * new_current_property);
  void selectionHasChanged();

private:
  /// Recursively write full names of properties which are expanded in this view to the Config.
  void saveExpandedEntries(
    Config config, const QModelIndex & parent_index,
    const QString & prefix) const;

  /// Recursively expand entries whose full names appear in expanded_full_names.
  void expandEntries(
    const QSet<QString> & expanded_full_names,
    const QModelIndex & parent_index,
    const QString & prefix);

  PropertyTreeModel * model_;
  SplitterHandle * splitter_handle_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
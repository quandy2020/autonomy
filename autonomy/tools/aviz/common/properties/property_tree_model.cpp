/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www/apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/tools/aviz/common/properties/property_tree_model.hpp"

#include "autonomy/tools/aviz/common/properties/property.hpp"

namespace aviz {
namespace common {
namespace properties {

PropertyTreeModel::PropertyTreeModel(Property* root_property, QObject* parent)
    : QAbstractItemModel(parent), root_property_(root_property) {
  if (root_property_) {
    root_property_->setModel(this);
  }
}

PropertyTreeModel::~PropertyTreeModel() {
  if (root_property_) {
    root_property_->setModel(nullptr);
    delete root_property_;
  }
}

void PropertyTreeModel::beginInsert(Property* parent, int index) {
  QModelIndex parent_index =
      parent == root_property_ ? QModelIndex() : createIndex(parent->rowNumberInParent(), 0, parent);
  beginInsertRows(parent_index, index, index);
}

void PropertyTreeModel::endInsert() { endInsertRows(); }

void PropertyTreeModel::beginRemove(Property* parent, int start_index, int count) {
  QModelIndex parent_index =
      parent == root_property_ ? QModelIndex() : createIndex(parent->rowNumberInParent(), 0, parent);
  beginRemoveRows(parent_index, start_index, start_index + count - 1);
}

void PropertyTreeModel::endRemove() { endRemoveRows(); }

void PropertyTreeModel::emitDataChanged(Property* property) {
  QModelIndex left = createIndex(property->rowNumberInParent(), 0, property);
  QModelIndex right = createIndex(property->rowNumberInParent(), 1, property);
  Q_EMIT dataChanged(left, right);
}

void PropertyTreeModel::emitPropertyHiddenChanged(Property* property) {
  Q_UNUSED(property);
  // Can be implemented later if needed
}

void PropertyTreeModel::expandProperty(Property* property) {
  Q_UNUSED(property);
  // Can be implemented later if needed
}

void PropertyTreeModel::collapseProperty(Property* property) {
  Q_UNUSED(property);
  // Can be implemented later if needed
}

QVariant PropertyTreeModel::data(const QModelIndex& index, int role) const {
  if (!index.isValid()) {
    return QVariant();
  }

  Property* property = static_cast<Property*>(index.internalPointer());
  if (!property) {
    return QVariant();
  }

  return property->getViewData(index.column(), role);
}

QVariant PropertyTreeModel::headerData(int section, Qt::Orientation orientation, int role) const {
  Q_UNUSED(orientation);
  if (role == Qt::DisplayRole) {
    if (section == 0) {
      return "Property";
    } else if (section == 1) {
      return "Value";
    }
  }
  return QVariant();
}

QModelIndex PropertyTreeModel::index(int row, int column, const QModelIndex& parent) const {
  if (!hasIndex(row, column, parent)) {
    return QModelIndex();
  }

  Property* parent_property;
  if (!parent.isValid()) {
    parent_property = root_property_;
  } else {
    parent_property = static_cast<Property*>(parent.internalPointer());
  }

  if (!parent_property) {
    return QModelIndex();
  }

  Property* child_property = parent_property->childAt(row);
  if (child_property) {
    return createIndex(row, column, child_property);
  }

  return QModelIndex();
}

QModelIndex PropertyTreeModel::parent(const QModelIndex& index) const {
  if (!index.isValid()) {
    return QModelIndex();
  }

  Property* child_property = static_cast<Property*>(index.internalPointer());
  if (!child_property) {
    return QModelIndex();
  }

  Property* parent_property = child_property->getParent();
  if (parent_property == root_property_ || !parent_property) {
    return QModelIndex();
  }

  return createIndex(parent_property->rowNumberInParent(), 0, parent_property);
}

int PropertyTreeModel::rowCount(const QModelIndex& parent) const {
  if (parent.column() > 0) {
    return 0;
  }

  Property* parent_property;
  if (!parent.isValid()) {
    parent_property = root_property_;
  } else {
    parent_property = static_cast<Property*>(parent.internalPointer());
  }

  if (!parent_property) {
    return 0;
  }

  return parent_property->numChildren();
}

int PropertyTreeModel::columnCount(const QModelIndex& parent) const {
  Q_UNUSED(parent);
  return 2;  // Name and Value columns
}

bool PropertyTreeModel::setData(const QModelIndex& index, const QVariant& value, int role) {
  if (!index.isValid() || role != Qt::EditRole) {
    return false;
  }

  Property* property = static_cast<Property*>(index.internalPointer());
  if (!property) {
    return false;
  }

  if (index.column() == 1) {  // Value column
    bool changed = property->setValue(value);
    if (changed) {
      Q_EMIT dataChanged(index, index);
    }
    return changed;
  }

  return false;
}

Qt::ItemFlags PropertyTreeModel::flags(const QModelIndex& index) const {
  if (!index.isValid()) {
    return Qt::NoItemFlags;
  }

  Property* property = static_cast<Property*>(index.internalPointer());
  if (!property) {
    return Qt::NoItemFlags;
  }

  return property->getViewFlags(index.column());
}

}  // namespace properties
}  // namespace common
}  // namespace aviz

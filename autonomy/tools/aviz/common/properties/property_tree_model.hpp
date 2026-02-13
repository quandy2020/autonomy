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

#ifndef AVIZ_COMMON__PROPERTIES__PROPERTY_TREE_MODEL_HPP_
#define AVIZ_COMMON__PROPERTIES__PROPERTY_TREE_MODEL_HPP_

#include <QAbstractItemModel>

namespace aviz {
namespace common {
namespace properties {

class Property;

/// Forward declaration - full implementation can be added later
class PropertyTreeModel : public QAbstractItemModel
{
    Q_OBJECT

public:
    explicit PropertyTreeModel(Property* root_property, QObject* parent = nullptr);
    virtual ~PropertyTreeModel();

    // Minimal interface for Property class to use
    void beginInsert(Property* parent, int index);
    void endInsert();
    void beginRemove(Property* parent, int start_index, int count);
    void endRemove();
    void emitDataChanged(Property* property);
    void emitPropertyHiddenChanged(Property* property);
    void expandProperty(Property* property);
    void collapseProperty(Property* property);

    // QAbstractItemModel interface
    QVariant data(const QModelIndex& index, int role) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;

private:
    Property* root_property_;
};

}  // namespace properties
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__PROPERTIES__PROPERTY_TREE_MODEL_HPP_

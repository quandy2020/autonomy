/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/table/table_model.hpp"

#include <limits>
#include <utility>

namespace autoviz::table_panel {

TableModel::TableModel(QObject* parent) : QAbstractTableModel(parent) {}

int TableModel::rowCount(const QModelIndex& parent) const {
  return parent.isValid() ? 0 : snapshot_.row_count;
}

int TableModel::columnCount(const QModelIndex& parent) const {
  return parent.isValid() ? 0 : static_cast<int>(snapshot_.headers.size());
}

QVariant TableModel::data(const QModelIndex& index, int role) const {
  if (!index.isValid() || index.row() < 0 || index.column() < 0 ||
      index.row() >= snapshot_.row_count ||
      index.column() >= static_cast<int>(snapshot_.headers.size())) {
    return {};
  }
  if (role != Qt::DisplayRole && role != Qt::EditRole &&
      role != Qt::ToolTipRole) {
    return {};
  }
  const qsizetype offset =
      static_cast<qsizetype>(index.row()) * snapshot_.headers.size() +
      index.column();
  return snapshot_.cells.at(offset);
}

QVariant TableModel::headerData(int section, Qt::Orientation orientation,
                                int role) const {
  if (role != Qt::DisplayRole || section < 0) {
    return {};
  }
  if (orientation == Qt::Horizontal) {
    return section < static_cast<int>(snapshot_.headers.size())
               ? snapshot_.headers.at(section)
               : QVariant();
  }
  return section < snapshot_.row_count ? QVariant(section + 1) : QVariant();
}

Qt::ItemFlags TableModel::flags(const QModelIndex& index) const {
  return index.isValid() ? Qt::ItemIsEnabled | Qt::ItemIsSelectable
                         : Qt::NoItemFlags;
}

bool TableModel::Replace(TableSnapshot snapshot) {
  if (snapshot.row_count < 0) {
    return false;
  }
  if (snapshot.headers.isEmpty()) {
    if (snapshot.row_count != 0 || !snapshot.cells.isEmpty()) {
      return false;
    }
    Clear();
    return true;
  }
  const qsizetype columns = snapshot.headers.size();
  if (snapshot.row_count > std::numeric_limits<qsizetype>::max() / columns) {
    return false;
  }
  const qsizetype expected =
      static_cast<qsizetype>(snapshot.row_count) * columns;
  if (snapshot.cells.size() != expected) {
    return false;
  }

  beginResetModel();
  snapshot_ = std::move(snapshot);
  endResetModel();
  return true;
}

void TableModel::Clear() {
  if (snapshot_.row_count == 0 && snapshot_.headers.isEmpty() &&
      snapshot_.cells.isEmpty()) {
    return;
  }
  beginResetModel();
  snapshot_ = TableSnapshot{};
  endResetModel();
}

}  // namespace autoviz::table_panel

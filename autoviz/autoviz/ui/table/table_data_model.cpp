/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/table/table_data_model.hpp"

#include <algorithm>

namespace autoviz {
namespace table {
namespace {

bool IsNumericLike(const QString& text, double* value) {
  if (text.isEmpty()) {
    return false;
  }
  bool ok = false;
  const double parsed = text.toDouble(&ok);
  if (ok) {
    *value = parsed;
    return true;
  }
  return false;
}

int CompareValues(const QString& left, const QString& right) {
  double left_num = 0.0;
  double right_num = 0.0;
  const bool left_numeric = IsNumericLike(left, &left_num);
  const bool right_numeric = IsNumericLike(right, &right_num);
  if (left_numeric && right_numeric) {
    if (left_num < right_num) {
      return -1;
    }
    if (left_num > right_num) {
      return 1;
    }
    return 0;
  }
  return QString::compare(left, right, Qt::CaseInsensitive);
}

}  // namespace

TableDataModel::TableDataModel(QObject* parent) : QAbstractTableModel(parent) {}

void TableDataModel::setTableData(const TableData& data) {
  beginResetModel();
  data_ = data;
  row_order_.resize(static_cast<int>(data_.rows.size()));
  for (int i = 0; i < row_order_.size(); ++i) {
    row_order_[i] = i;
  }
  sort_keys_.clear();
  endResetModel();
}

void TableDataModel::clearData() {
  beginResetModel();
  data_ = TableData{};
  row_order_.clear();
  sort_keys_.clear();
  endResetModel();
}

int TableDataModel::rowCount(const QModelIndex& parent) const {
  if (parent.isValid()) {
    return 0;
  }
  return row_order_.size();
}

int TableDataModel::columnCount(const QModelIndex& parent) const {
  if (parent.isValid()) {
    return 0;
  }
  return static_cast<int>(data_.columns.size());
}

QVariant TableDataModel::data(const QModelIndex& index, int role) const {
  if (!index.isValid() || role != Qt::DisplayRole) {
    return {};
  }
  const int source_row = row_order_.value(index.row(), -1);
  if (source_row < 0 || source_row >= static_cast<int>(data_.rows.size())) {
    return {};
  }
  const std::vector<QString>& row = data_.rows.at(static_cast<size_t>(source_row));
  if (index.column() < 0 || index.column() >= static_cast<int>(row.size())) {
    return {};
  }
  return row.at(static_cast<size_t>(index.column()));
}

QVariant TableDataModel::headerData(int section, Qt::Orientation orientation,
                                    int role) const {
  if (role != Qt::DisplayRole || orientation != Qt::Horizontal) {
    return {};
  }
  if (section < 0 || section >= static_cast<int>(data_.columns.size())) {
    return {};
  }
  return data_.columns.at(static_cast<size_t>(section)).name;
}

bool TableDataModel::CompareRows(const std::vector<QString>& left,
                                 const std::vector<QString>& right,
                                 const QVector<SortKey>& keys) {
  for (const SortKey& key : keys) {
    if (key.column < 0 || key.column >= static_cast<int>(left.size()) ||
        key.column >= static_cast<int>(right.size())) {
      continue;
    }
    const int cmp =
        CompareValues(left.at(static_cast<size_t>(key.column)),
                      right.at(static_cast<size_t>(key.column)));
    if (cmp != 0) {
      return key.order == Qt::AscendingOrder ? cmp < 0 : cmp > 0;
    }
  }
  return false;
}

void TableDataModel::applySort(int column, bool append, Qt::SortOrder order) {
  if (column < 0 || data_.rows.empty()) {
    return;
  }
  layoutAboutToBeChanged();
  if (append) {
    for (int i = 0; i < sort_keys_.size(); ++i) {
      if (sort_keys_[i].column == column) {
        sort_keys_.removeAt(i);
        break;
      }
    }
    sort_keys_.push_back({column, order});
  } else {
    sort_keys_ = {{column, order}};
  }
  if (sort_keys_.isEmpty()) {
    for (int i = 0; i < row_order_.size(); ++i) {
      row_order_[i] = i;
    }
  } else {
    std::stable_sort(row_order_.begin(), row_order_.end(),
                     [this](int left_index, int right_index) {
                       return CompareRows(
                           data_.rows.at(static_cast<size_t>(left_index)),
                           data_.rows.at(static_cast<size_t>(right_index)),
                           sort_keys_);
                     });
  }
  layoutChanged();
}

void TableDataModel::sort(int column, Qt::SortOrder order) {
  applySort(column, false, order);
}

int TableDataModel::sortColumn() const {
  return sort_keys_.isEmpty() ? -1 : sort_keys_.last().column;
}

Qt::SortOrder TableDataModel::sortOrder() const {
  return sort_keys_.isEmpty() ? Qt::AscendingOrder : sort_keys_.last().order;
}

}  // namespace table
}  // namespace autoviz

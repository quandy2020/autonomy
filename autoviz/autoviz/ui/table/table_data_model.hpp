/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractTableModel>
#include <QVector>

#include "autoviz/ui/table/table_field_extractor.hpp"

namespace autoviz {
namespace table {

class TableDataModel : public QAbstractTableModel {
  Q_OBJECT

 public:
  explicit TableDataModel(QObject* parent = nullptr);

  void setTableData(const TableData& data);
  void clearData();

  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  int columnCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
  QVariant headerData(int section, Qt::Orientation orientation,
                      int role = Qt::DisplayRole) const override;
  void sort(int column, Qt::SortOrder order = Qt::AscendingOrder) override;
  void applySort(int column, bool append, Qt::SortOrder order);

  int sortColumn() const;
  Qt::SortOrder sortOrder() const;

 private:
  struct SortKey {
    int column = 0;
    Qt::SortOrder order = Qt::AscendingOrder;
  };

  static bool CompareRows(const std::vector<QString>& left,
                          const std::vector<QString>& right,
                          const QVector<SortKey>& keys);

  TableData data_;
  QVector<int> row_order_;
  QVector<SortKey> sort_keys_;
};

}  // namespace table
}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractTableModel>
#include <QStringList>
#include <QVariant>
#include <QVector>

namespace autoviz::table_panel {

// Immutable-by-convention row-major snapshot used for one atomic table update.
struct TableSnapshot {
  QStringList headers;
  QVector<QString> cells;
  int row_count = 0;
  bool capped = false;
};

// Allocation-conscious model for live protobuf arrays. Replacing a snapshot
// emits one reset instead of allocating one QTableWidgetItem per cell.
class TableModel final : public QAbstractTableModel {
 public:
  explicit TableModel(QObject* parent = nullptr);

  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  int columnCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& index,
                int role = Qt::DisplayRole) const override;
  QVariant headerData(int section, Qt::Orientation orientation,
                      int role = Qt::DisplayRole) const override;
  Qt::ItemFlags flags(const QModelIndex& index) const override;

  bool Replace(TableSnapshot snapshot);
  void Clear();

 private:
  TableSnapshot snapshot_;
};

}  // namespace autoviz::table_panel

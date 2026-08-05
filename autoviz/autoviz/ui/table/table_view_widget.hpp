/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/table/table_field_extractor.hpp"

class QLabel;
class QTableView;

namespace autoviz {
namespace table {

class TableDataModel;

class TableViewWidget : public QWidget {
  Q_OBJECT

 public:
  explicit TableViewWidget(QWidget* parent = nullptr);

  void setTableData(const TableData& data);
  void clearData();
  void setStatusText(const QString& text);

 private:
  TableDataModel* model_ = nullptr;
  QTableView* table_ = nullptr;
  QLabel* empty_label_ = nullptr;
};

}  // namespace table
}  // namespace autoviz

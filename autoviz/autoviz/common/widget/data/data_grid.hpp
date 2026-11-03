/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QByteArray>
#include <QPointer>
#include <QString>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QAbstractItemModel;
class QSortFilterProxyModel;
class QTableView;

namespace autoviz::widget {

// Virtualized table facade with filtering, sorting, persisted columns, and a
// nested batch-update guard. It never installs per-cell widgets.
class DataGrid final : public QWidget {
  Q_OBJECT

 public:
  explicit DataGrid(QWidget* parent = nullptr);

  void SetSourceModel(QAbstractItemModel* model);
  QAbstractItemModel* source_model() const;
  QSortFilterProxyModel* proxy_model() const noexcept { return proxy_model_; }
  QTableView* view() const noexcept { return view_; }

  void SetFilterText(const QString& text);
  QString filter_text() const;
  void SetFilterColumn(int column);
  void SetColumnVisible(int column, bool visible);
  bool IsColumnVisible(int column) const;
  QByteArray SaveHeaderState() const;
  bool RestoreHeaderState(const QByteArray& state);
  void SetTone(style::Tone tone);

  void BeginBatchUpdate();
  void EndBatchUpdate();

 signals:
  void FilterChanged(const QString& text);

 private:
  QTableView* view_ = nullptr;
  QSortFilterProxyModel* proxy_model_ = nullptr;
  style::Tone tone_ = style::Tone::Shell;
  int batch_depth_ = 0;
  bool dynamic_sort_filter_before_batch_ = true;
  QString filter_text_;
};

class ScopedDataGridUpdate final {
 public:
  explicit ScopedDataGridUpdate(DataGrid* grid) : grid_(grid) {
    if (grid_ != nullptr) {
      grid_->BeginBatchUpdate();
    }
  }
  ~ScopedDataGridUpdate() {
    if (grid_ != nullptr) {
      grid_->EndBatchUpdate();
    }
  }

  ScopedDataGridUpdate(const ScopedDataGridUpdate&) = delete;
  ScopedDataGridUpdate& operator=(const ScopedDataGridUpdate&) = delete;

 private:
  QPointer<DataGrid> grid_;
};

}  // namespace autoviz::widget

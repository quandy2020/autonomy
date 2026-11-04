/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/data/data_grid.hpp"

#include <QAbstractItemModel>
#include <QFrame>
#include <QHeaderView>
#include <QRegularExpression>
#include <QSortFilterProxyModel>
#include <QTableView>
#include <QVBoxLayout>
#include <QVariant>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {

DataGrid::DataGrid(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizDataGrid"));
  setProperty("autovizRole", QStringLiteral("dataGrid"));
  setAccessibleName(tr("Data grid"));

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  proxy_model_ = new QSortFilterProxyModel(this);
  proxy_model_->setDynamicSortFilter(true);
  proxy_model_->setFilterCaseSensitivity(Qt::CaseInsensitive);
  proxy_model_->setSortCaseSensitivity(Qt::CaseInsensitive);
  proxy_model_->setFilterKeyColumn(-1);

  view_ = new QTableView(this);
  view_->setObjectName(QStringLiteral("AutovizDataGridView"));
  view_->setModel(proxy_model_);
  view_->setSortingEnabled(true);
  view_->setAlternatingRowColors(true);
  view_->setShowGrid(false);
  view_->setFrameShape(QFrame::NoFrame);
  view_->setCornerButtonEnabled(false);
  view_->setWordWrap(false);
  view_->setSelectionBehavior(QAbstractItemView::SelectRows);
  view_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  view_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
  view_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
  view_->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  view_->verticalHeader()->setDefaultSectionSize(
      GetThemeTokens(this, tone_).control_height_small);
  view_->horizontalHeader()->setSectionsMovable(true);
  view_->horizontalHeader()->setHighlightSections(false);
  view_->horizontalHeader()->setStretchLastSection(true);
  layout->addWidget(view_);
  ApplyWidgetTheme(this, tone_);
}

void DataGrid::SetSourceModel(QAbstractItemModel* model) {
  proxy_model_->setSourceModel(model);
}

QAbstractItemModel* DataGrid::source_model() const {
  return proxy_model_->sourceModel();
}

void DataGrid::SetFilterText(const QString& text) {
  if (filter_text_ == text) {
    return;
  }
  filter_text_ = text;
  proxy_model_->setFilterRegularExpression(
      QRegularExpression(QRegularExpression::escape(text),
                         QRegularExpression::CaseInsensitiveOption));
  emit FilterChanged(text);
}

QString DataGrid::filter_text() const { return filter_text_; }

void DataGrid::SetFilterColumn(int column) {
  proxy_model_->setFilterKeyColumn(column);
}

void DataGrid::SetColumnVisible(int column, bool visible) {
  if (column < 0 || column >= view_->model()->columnCount()) {
    return;
  }
  view_->setColumnHidden(column, !visible);
}

bool DataGrid::IsColumnVisible(int column) const {
  return column >= 0 && column < view_->model()->columnCount() &&
         !view_->isColumnHidden(column);
}

QByteArray DataGrid::SaveHeaderState() const {
  return view_->horizontalHeader()->saveState();
}

bool DataGrid::RestoreHeaderState(const QByteArray& state) {
  return !state.isEmpty() && view_->horizontalHeader()->restoreState(state);
}

void DataGrid::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  view_->verticalHeader()->setDefaultSectionSize(
      GetThemeTokens(this, tone_).control_height_small);
  ApplyWidgetTheme(this, tone_);
}

void DataGrid::BeginBatchUpdate() {
  ++batch_depth_;
  if (batch_depth_ != 1) {
    return;
  }
  dynamic_sort_filter_before_batch_ = proxy_model_->dynamicSortFilter();
  proxy_model_->setDynamicSortFilter(false);
  view_->setUpdatesEnabled(false);
}

void DataGrid::EndBatchUpdate() {
  if (batch_depth_ <= 0) {
    return;
  }
  --batch_depth_;
  if (batch_depth_ != 0) {
    return;
  }
  proxy_model_->setDynamicSortFilter(dynamic_sort_filter_before_batch_);
  proxy_model_->invalidate();
  view_->setUpdatesEnabled(true);
  view_->viewport()->update();
}

}  // namespace autoviz::widget

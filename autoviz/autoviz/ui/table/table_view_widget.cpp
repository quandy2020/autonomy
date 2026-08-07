/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/table/table_view_widget.hpp"

#include <algorithm>

#include <QFrame>
#include <QHeaderView>
#include <QLabel>
#include <QMouseEvent>
#include <QTableView>
#include <QVBoxLayout>

#include "autoviz/ui/table/table_data_model.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace table {
namespace {

class TableSortHeaderView : public QHeaderView {
 public:
  explicit TableSortHeaderView(Qt::Orientation orientation, TableDataModel* model,
                               QWidget* parent = nullptr)
      : QHeaderView(orientation, parent), model_(model) {}

 protected:
  void mousePressEvent(QMouseEvent* event) override {
    if (event == nullptr || model_ == nullptr || orientation() != Qt::Horizontal) {
      QHeaderView::mousePressEvent(event);
      return;
    }
    const int section = logicalIndexAt(event->position().toPoint());
    if (section >= 0 && event->button() == Qt::LeftButton) {
      const Qt::SortOrder order =
          (model_->sortOrder() == Qt::AscendingOrder &&
           model_->sortColumn() == section)
              ? Qt::DescendingOrder
              : Qt::AscendingOrder;
      model_->applySort(section, event->modifiers() & Qt::ShiftModifier, order);
      event->accept();
      return;
    }
    QHeaderView::mousePressEvent(event);
  }

 private:
  TableDataModel* model_ = nullptr;
};

}  // namespace

TableViewWidget::TableViewWidget(QWidget* parent) : QWidget(parent) {
  ApplyPanelShell(this);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  model_ = new TableDataModel(this);
  table_ = new QTableView(this);
  table_->setModel(model_);
  table_->setFrameShape(QFrame::NoFrame);
  table_->setAlternatingRowColors(true);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSortingEnabled(false);
  table_->setWordWrap(false);
  table_->setTextElideMode(Qt::ElideRight);
  table_->verticalHeader()->setVisible(false);
  table_->verticalHeader()->setDefaultSectionSize(22);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->horizontalHeader()->setSectionsClickable(true);
  table_->horizontalHeader()->setHighlightSections(true);
  table_->setStyleSheet(
      QStringLiteral(
          "QTableView {"
          "  background: palette(base);"
          "  alternate-background-color: palette(alternate-base);"
          "  gridline-color: palette(midlight);"
          "  border: none;"
          "  selection-background-color: palette(highlight);"
          "}"
          "QHeaderView::section {"
          "  background: palette(alternate-base);"
          "  color: palette(text);"
          "  padding: 3px 6px;"
          "  border: none;"
          "  border-right: 1px solid palette(midlight);"
          "  border-bottom: 1px solid palette(midlight);"
          "  font-weight: 600;"
          "}"));

  auto* header = new TableSortHeaderView(Qt::Horizontal, model_, table_);
  table_->setHorizontalHeader(header);

  empty_label_ = new QLabel(tr("Drop a topic or array field here"), this);
  empty_label_->setAlignment(Qt::AlignCenter);
  StyleHintLabel(empty_label_);
  empty_label_->setWordWrap(true);

  root->addWidget(table_, 1);
  root->addWidget(empty_label_, 1);
  empty_label_->hide();
}

void TableViewWidget::setTableData(const TableData& data) {
  model_->setTableData(data);
  const bool has_rows = !data.rows.empty();
  table_->setVisible(has_rows);
  empty_label_->setVisible(!has_rows);
  if (has_rows) {
    table_->resizeColumnsToContents();
    for (int col = 0; col < model_->columnCount(); ++col) {
      const int width = table_->columnWidth(col);
      table_->setColumnWidth(col, std::min(std::max(width, 48), 320));
    }
  }
}

void TableViewWidget::clearData() {
  model_->clearData();
  table_->setVisible(false);
  empty_label_->setVisible(true);
}

void TableViewWidget::setStatusText(const QString& text) {
  if (empty_label_->isVisible()) {
    empty_label_->setText(text);
  }
}

}  // namespace table
}  // namespace autoviz

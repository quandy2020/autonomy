/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_blackboard_view.hpp"

#include <QHeaderView>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace autoviz {
namespace behavior_tree {
namespace {

constexpr char kBg[] = "#f8f9fb";
constexpr char kText[] = "#0f172a";
constexpr char kTextMuted[] = "#64748b";
constexpr char kBorder[] = "#e2e8f0";

QString TableStyleSheet() {
  return QStringLiteral(
      "QTableWidget {"
      "  background: %1; color: %2; border: none; outline: none;"
      "  gridline-color: %3; font-size: 12px;"
      "}"
      "QTableWidget::item { padding: 4px 8px; }"
      "QTableWidget::item:selected {"
      "  background: rgba(8,145,178,0.14); color: %2;"
      "}"
      "QHeaderView::section {"
      "  background: #eef2f7; color: %4;"
      "  border: none; border-bottom: 1px solid %3;"
      "  padding: 6px 8px; font-size: 11px; font-weight: 700;"
      "}")
      .arg(QLatin1String(kBg), QLatin1String(kText), QLatin1String(kBorder),
           QLatin1String(kTextMuted));
}

}  // namespace

BtBlackboardView::BtBlackboardView(QWidget* parent) : QWidget(parent) {
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(QStringLiteral("background: %1;").arg(QLatin1String(kBg)));

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  table_ = new QTableWidget(this);
  table_->setColumnCount(2);
  table_->setHorizontalHeaderLabels({tr("Key"), tr("Value")});
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  table_->verticalHeader()->setVisible(false);
  table_->setShowGrid(true);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setAlternatingRowColors(true);
  table_->setStyleSheet(TableStyleSheet());
  layout->addWidget(table_);
}

void BtBlackboardView::setEntries(const QHash<QString, QString>& entries) {
  QStringList keys = entries.keys();
  keys.sort(Qt::CaseInsensitive);
  table_->setRowCount(keys.size());
  for (int row = 0; row < keys.size(); ++row) {
    const QString& key = keys.at(row);
    table_->setItem(row, 0, new QTableWidgetItem(key));
    table_->setItem(row, 1, new QTableWidgetItem(entries.value(key)));
  }
}

void BtBlackboardView::clear() {
  table_->setRowCount(0);
}

}  // namespace behavior_tree
}  // namespace autoviz

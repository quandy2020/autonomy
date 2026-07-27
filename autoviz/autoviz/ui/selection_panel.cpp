/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/selection_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace autoviz {

SelectionPanel::SelectionPanel(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel(tr("Selected Points"), this));
  list_ = new QListWidget(this);
  layout->addWidget(list_);
}

void SelectionPanel::setSelections(
    const std::vector<common::SelectionEntry>& entries) {
  list_->clear();
  if (entries.empty()) {
    list_->addItem(tr("(none)"));
    return;
  }
  for (const auto& entry : entries) {
    const QVector3D p = entry.position;
    QString label;
    if (entry.display_name.empty()) {
      label = QStringLiteral("Point: (%1, %2, %3)")
                  .arg(p.x(), 0, 'f', 3)
                  .arg(p.y(), 0, 'f', 3)
                  .arg(p.z(), 0, 'f', 3);
    } else if (entry.display_type.empty()) {
      label = QStringLiteral("%1: (%2, %3, %4)")
                  .arg(QString::fromStdString(entry.display_name))
                  .arg(p.x(), 0, 'f', 3)
                  .arg(p.y(), 0, 'f', 3)
                  .arg(p.z(), 0, 'f', 3);
    } else {
      label = QStringLiteral("%1 [%2]: (%3, %4, %5)")
                  .arg(QString::fromStdString(entry.display_name))
                  .arg(QString::fromStdString(entry.display_type))
                  .arg(p.x(), 0, 'f', 3)
                  .arg(p.y(), 0, 'f', 3)
                  .arg(p.z(), 0, 'f', 3);
    }
    list_->addItem(label);
  }
}

}  // namespace autoviz

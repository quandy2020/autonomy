/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/inspector/selection_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPaintEvent>
#include <QVector3D>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {

SelectionPanel::SelectionPanel(common::VisualizationManager* manager,
                             QWidget* parent)
    : manager_(manager), QWidget(parent) {
  setObjectName(QStringLiteral("Selection/SelectionPanel"));
  setAttribute(Qt::WA_StyledBackground, false);
  setAutoFillBackground(false);
  ApplyPanelShell(this);

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(0);

  QHBoxLayout* toolbar_layout = nullptr;
  auto* toolbar = MakePanelToolbar(this, &toolbar_layout);
  auto* title = new QLabel(tr("Selected Points"), toolbar);
  StyleSectionTitle(title);
  toolbar_layout->addWidget(title, 1);
  layout->addWidget(toolbar);

  list_ = new QListWidget(this);
  StyleFrostedPanelList(list_);
  layout->addWidget(list_, 1);
}

void SelectionPanel::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  PaintPanelFrostedCard(painter, rect(), 14.0);
}

void SelectionPanel::setSelections(
    const std::vector<common::SelectionEntry>& entries) {
  entries_ = entries;
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

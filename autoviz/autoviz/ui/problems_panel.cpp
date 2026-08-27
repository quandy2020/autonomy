/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/problems_panel.hpp"

#include <QPlainTextEdit>
#include <QVBoxLayout>

#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {

ProblemsPanel::ProblemsPanel(QWidget* parent) : QWidget(parent) {
  ApplyPanelShell(this);
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  content_ = new QPlainTextEdit(this);
  content_->setReadOnly(true);
  content_->setPlaceholderText(tr("No problems detected."));
  content_->setStyleSheet(QStringLiteral(
      "QPlainTextEdit {"
      "  background: #ffffff; color: #1e293b;"
      "  border: none; padding: 8px; font-size: 12px;"
      "}"));
  layout->addWidget(content_);
  refresh();
}

void ProblemsPanel::refresh() {
  if (content_ == nullptr) {
    return;
  }
  content_->setPlainText(tr("No problems detected."));
}

}  // namespace autoviz

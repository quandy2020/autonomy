/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/problems_panel.hpp"

#include <QPlainTextEdit>
#include <QVBoxLayout>

namespace autoviz {

ProblemsPanel::ProblemsPanel(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  content_ = new QPlainTextEdit(this);
  content_->setReadOnly(true);
  content_->setPlaceholderText(tr("No problems detected."));
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

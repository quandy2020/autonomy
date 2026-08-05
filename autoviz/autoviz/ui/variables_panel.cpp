/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/variables_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>

namespace autoviz {

VariablesPanel::VariablesPanel(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  auto* label = new QLabel(
      tr("Global variables can be referenced in message paths. "
         "Variable editing is planned."),
      this);
  label->setWordWrap(true);
  layout->addWidget(label);
  layout->addStretch();
}

}  // namespace autoviz

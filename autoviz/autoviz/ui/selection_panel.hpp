/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <vector>

#include <QComboBox>
#include <QListWidget>
#include <QPushButton>
#include <QWidget>

#include "autoviz/common/selection.hpp"

namespace autoviz {

namespace common {
class VisualizationManager;
}

class SelectionPanel : public QWidget {
  Q_OBJECT

 public:
  explicit SelectionPanel(common::VisualizationManager* manager,
                          QWidget* parent = nullptr);

  void setSelections(const std::vector<common::SelectionEntry>& entries);

 private slots:
  void onSetVariableFromSelection();
  void onVariablesChanged();

 private:
  void rebuildVariableList();

  common::VisualizationManager* manager_ = nullptr;
  QListWidget* list_ = nullptr;
  QComboBox* variable_combo_ = nullptr;
  QComboBox* axis_combo_ = nullptr;
  QPushButton* set_button_ = nullptr;
  std::vector<common::SelectionEntry> entries_;
};

}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/selection_panel.hpp"

#include <QComboBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/variables/variable_store.hpp"
#include "autoviz/variables/variable_types.hpp"

namespace autoviz {

SelectionPanel::SelectionPanel(common::VisualizationManager* manager,
                             QWidget* parent)
    : manager_(manager), QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel(tr("Selected Points"), this));
  list_ = new QListWidget(this);
  layout->addWidget(list_, 1);

  auto* variable_box = new QWidget(this);
  auto* form = new QFormLayout(variable_box);
  variable_combo_ = new QComboBox(variable_box);
  axis_combo_ = new QComboBox(variable_box);
  axis_combo_->addItem(tr("X"), 0);
  axis_combo_->addItem(tr("Y"), 1);
  axis_combo_->addItem(tr("Z"), 2);
  set_button_ = new QPushButton(tr("Set variable from selection"), variable_box);
  form->addRow(tr("Variable"), variable_combo_);
  form->addRow(tr("Axis"), axis_combo_);
  form->addRow(set_button_);
  layout->addWidget(variable_box);

  connect(set_button_, &QPushButton::clicked, this,
          &SelectionPanel::onSetVariableFromSelection);
  if (manager_ != nullptr) {
    connect(&manager_->variableStore(),
            &variables::VariableStore::variablesChanged, this,
            &SelectionPanel::onVariablesChanged);
    rebuildVariableList();
  }
}

void SelectionPanel::rebuildVariableList() {
  if (manager_ == nullptr || variable_combo_ == nullptr) {
    return;
  }
  const QString current = variable_combo_->currentData().toString();
  variable_combo_->clear();
  for (const variables::VariableEntry& entry :
       manager_->variableStore().variables()) {
    if (entry.type != variables::VariableType::kNumber) {
      continue;
    }
    variable_combo_->addItem(entry.name, entry.name);
  }
  const int index = variable_combo_->findData(current);
  variable_combo_->setCurrentIndex(index >= 0 ? index : 0);
  set_button_->setEnabled(variable_combo_->count() > 0 && !entries_.empty());
}

void SelectionPanel::onVariablesChanged() { rebuildVariableList(); }

void SelectionPanel::setSelections(
    const std::vector<common::SelectionEntry>& entries) {
  entries_ = entries;
  list_->clear();
  if (entries.empty()) {
    list_->addItem(tr("(none)"));
    set_button_->setEnabled(false);
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
  set_button_->setEnabled(variable_combo_ != nullptr &&
                          variable_combo_->count() > 0);
}

void SelectionPanel::onSetVariableFromSelection() {
  if (manager_ == nullptr || entries_.empty() || variable_combo_ == nullptr ||
      axis_combo_ == nullptr) {
    return;
  }
  const QString variable_name = variable_combo_->currentData().toString();
  if (variable_name.isEmpty()) {
    return;
  }
  const QVector3D position = entries_.front().position;
  const int axis = axis_combo_->currentData().toInt();
  const double value =
      axis == 0 ? position.x() : axis == 1 ? position.y() : position.z();
  manager_->variableStore().setVariable(variable_name,
                                        variables::VariableType::kNumber, value);
}

}  // namespace autoviz

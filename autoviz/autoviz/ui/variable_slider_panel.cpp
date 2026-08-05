/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/variable_slider_panel.hpp"

#include <algorithm>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/variables/variable_store.hpp"
#include "autoviz/variables/variable_types.hpp"

namespace autoviz {

VariableSliderPanel::VariableSliderPanel(common::VisualizationManager* manager,
                                         QWidget* parent)
    : manager_(manager), QWidget(parent) {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(8);

  hint_label_ = new QLabel(
      tr("Adjust a numeric global variable with a slider."), this);
  hint_label_->setWordWrap(true);
  hint_label_->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 11px;"));
  root->addWidget(hint_label_);

  variable_combo_ = new QComboBox(this);
  root->addWidget(variable_combo_);

  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setRange(0, 1000);
  root->addWidget(slider_);

  value_spin_ = new QDoubleSpinBox(this);
  value_spin_->setDecimals(6);
  value_spin_->setRange(-1e12, 1e12);
  root->addWidget(value_spin_);

  auto* range_form = new QFormLayout();
  min_spin_ = new QDoubleSpinBox(this);
  max_spin_ = new QDoubleSpinBox(this);
  step_spin_ = new QDoubleSpinBox(this);
  min_spin_->setDecimals(4);
  max_spin_->setDecimals(4);
  step_spin_->setDecimals(4);
  min_spin_->setRange(-1e12, 1e12);
  max_spin_->setRange(-1e12, 1e12);
  step_spin_->setRange(1e-9, 1e12);
  min_spin_->setValue(0.0);
  max_spin_->setValue(100.0);
  step_spin_->setValue(1.0);
  range_form->addRow(tr("Min"), min_spin_);
  range_form->addRow(tr("Max"), max_spin_);
  range_form->addRow(tr("Step"), step_spin_);
  root->addLayout(range_form);
  root->addStretch();

  connect(variable_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &VariableSliderPanel::onVariableSelected);
  connect(slider_, &QSlider::valueChanged, this,
          &VariableSliderPanel::onSliderChanged);
  connect(value_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &VariableSliderPanel::onSpinChanged);
  connect(min_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &VariableSliderPanel::onRangeChanged);
  connect(max_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &VariableSliderPanel::onRangeChanged);
  connect(step_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &VariableSliderPanel::onRangeChanged);

  if (manager_ != nullptr) {
    connect(&manager_->variableStore(),
            &variables::VariableStore::variablesChanged, this,
            &VariableSliderPanel::onVariablesChanged);
    rebuildVariableList();
  }
}

void VariableSliderPanel::rebuildVariableList() {
  if (manager_ == nullptr || variable_combo_ == nullptr) {
    return;
  }
  syncing_ = true;
  variable_combo_->clear();
  const QVector<variables::VariableEntry> entries =
      manager_->variableStore().variables();
  for (const variables::VariableEntry& entry : entries) {
    if (entry.type != variables::VariableType::kNumber) {
      continue;
    }
    variable_combo_->addItem(entry.name, entry.name);
  }
  if (variable_combo_->count() == 0) {
    active_variable_.clear();
    slider_->setEnabled(false);
    value_spin_->setEnabled(false);
    hint_label_->setText(tr("Add a numeric global variable in the Variables panel."));
  } else {
    slider_->setEnabled(true);
    value_spin_->setEnabled(true);
    hint_label_->setText(tr("Adjust a numeric global variable with a slider."));
    int index = 0;
    if (!active_variable_.isEmpty()) {
      index = variable_combo_->findData(active_variable_);
      if (index < 0) {
        index = 0;
      }
    }
    variable_combo_->setCurrentIndex(index);
    onVariableSelected(index);
  }
  syncing_ = false;
}

void VariableSliderPanel::onVariablesChanged() {
  rebuildVariableList();
  syncFromStore();
}

void VariableSliderPanel::onVariableSelected(int index) {
  if (syncing_ || index < 0 || variable_combo_ == nullptr) {
    return;
  }
  active_variable_ = variable_combo_->itemData(index).toString();
  syncFromStore();
}

void VariableSliderPanel::syncFromStore() {
  if (manager_ == nullptr || active_variable_.isEmpty()) {
    return;
  }
  const auto entry = manager_->variableStore().variable(active_variable_);
  if (!entry.has_value() || entry->type != variables::VariableType::kNumber) {
    return;
  }
  syncing_ = true;
  const double value = entry->value.toDouble();
  const double min_value = min_spin_->value();
  const double max_value = max_spin_->value();
  const double step = std::max(step_spin_->value(), 1e-9);
  value_spin_->setSingleStep(step);
  value_spin_->setRange(min_value, max_value);
  value_spin_->setValue(std::clamp(value, min_value, max_value));
  const int slider_value =
      max_value > min_value
          ? static_cast<int>(std::lround(
                1000.0 * (value_spin_->value() - min_value) / (max_value - min_value)))
          : 0;
  slider_->blockSignals(true);
  slider_->setValue(std::clamp(slider_value, 0, 1000));
  slider_->blockSignals(false);
  syncing_ = false;
}

void VariableSliderPanel::pushToStore() {
  if (syncing_ || manager_ == nullptr || active_variable_.isEmpty()) {
    return;
  }
  manager_->variableStore().setVariable(
      active_variable_, variables::VariableType::kNumber, value_spin_->value());
}

void VariableSliderPanel::onSliderChanged(int value) {
  if (syncing_) {
    return;
  }
  const double min_value = min_spin_->value();
  const double max_value = max_spin_->value();
  if (max_value <= min_value) {
    return;
  }
  syncing_ = true;
  const double mapped =
      min_value + (max_value - min_value) * (static_cast<double>(value) / 1000.0);
  value_spin_->setValue(mapped);
  syncing_ = false;
  pushToStore();
}

void VariableSliderPanel::onSpinChanged(double) {
  if (syncing_) {
    return;
  }
  syncFromStore();
  pushToStore();
}

void VariableSliderPanel::onRangeChanged() {
  if (min_spin_->value() >= max_spin_->value()) {
    max_spin_->setValue(min_spin_->value() + 1.0);
  }
  syncFromStore();
}

}  // namespace autoviz

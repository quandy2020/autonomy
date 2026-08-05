/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QSlider;

namespace autoviz {

namespace common {
class VisualizationManager;
}

/** Foxglove-style slider bound to a numeric global variable. */
class VariableSliderPanel : public QWidget {
  Q_OBJECT

 public:
  explicit VariableSliderPanel(common::VisualizationManager* manager,
                               QWidget* parent = nullptr);

 private slots:
  void onVariablesChanged();
  void onVariableSelected(int index);
  void onSliderChanged(int value);
  void onSpinChanged(double value);
  void onRangeChanged();

 private:
  void rebuildVariableList();
  void syncFromStore();
  void pushToStore();

  common::VisualizationManager* manager_ = nullptr;
  QComboBox* variable_combo_ = nullptr;
  QSlider* slider_ = nullptr;
  QDoubleSpinBox* value_spin_ = nullptr;
  QDoubleSpinBox* min_spin_ = nullptr;
  QDoubleSpinBox* max_spin_ = nullptr;
  QDoubleSpinBox* step_spin_ = nullptr;
  QLabel* hint_label_ = nullptr;
  bool syncing_ = false;
  QString active_variable_;
};

}  // namespace autoviz

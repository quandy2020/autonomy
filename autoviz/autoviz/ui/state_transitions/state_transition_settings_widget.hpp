/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/state_transitions/state_transition_types.hpp"

class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLineEdit;
class QScrollArea;
class QVBoxLayout;

namespace autoviz {

namespace common {
class VisualizationManager;
}

namespace state_transitions {

class StateTransitionSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit StateTransitionSettingsWidget(common::VisualizationManager* manager,
                                       QWidget* parent = nullptr);

  StateTransitionPanelConfig config() const;
  void setConfig(const StateTransitionPanelConfig& config);
  void refreshChannels();

 signals:
  void configChanged();
  void addSeriesRequested();
  void removeSeriesRequested(int index);

 private slots:
  void emitConfigChanged();

 private:
  void rebuildSeriesEditors();
  QWidget* createSeriesEditor(int index);
  QWidget* createMappingRuleEditor(int series_index, int mapping_index);
  void updateAxisFieldVisibility();

  common::VisualizationManager* manager_ = nullptr;
  StateTransitionPanelConfig config_;
  QLineEdit* title_edit_ = nullptr;
  QComboBox* x_axis_mode_combo_ = nullptr;
  QDoubleSpinBox* window_spin_ = nullptr;
  QDoubleSpinBox* fixed_min_spin_ = nullptr;
  QDoubleSpinBox* fixed_max_spin_ = nullptr;
  QWidget* series_container_ = nullptr;
  QVBoxLayout* series_layout_ = nullptr;
  bool syncing_ = false;
};

}  // namespace state_transitions
}  // namespace autoviz

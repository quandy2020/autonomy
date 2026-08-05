/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/gauge/gauge_types.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QPushButton;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace gauge {

class GaugeSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit GaugeSettingsWidget(common::VisualizationManager* manager,
                               QWidget* parent = nullptr);

  GaugePanelConfig config() const;
  void setConfig(const GaugePanelConfig& config);
  void refreshChannels();

 signals:
  void configChanged();

 private:
  void emitConfigChanged();
  void pickGradientColor(bool start);
  void syncColorModeUi();

  common::VisualizationManager* manager_ = nullptr;
  GaugePanelConfig config_;
  QLineEdit* title_edit_ = nullptr;
  QComboBox* channel_combo_ = nullptr;
  QLineEdit* field_path_edit_ = nullptr;
  QDoubleSpinBox* min_spin_ = nullptr;
  QDoubleSpinBox* max_spin_ = nullptr;
  QComboBox* color_mode_combo_ = nullptr;
  QComboBox* color_map_combo_ = nullptr;
  QPushButton* gradient_start_button_ = nullptr;
  QPushButton* gradient_end_button_ = nullptr;
  QCheckBox* reverse_color_check_ = nullptr;
  QCheckBox* reverse_direction_check_ = nullptr;
  QWidget* gradient_body_ = nullptr;
  QWidget* color_map_body_ = nullptr;
};

}  // namespace gauge
}  // namespace autoviz

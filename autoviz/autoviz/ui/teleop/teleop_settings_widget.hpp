/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/teleop/teleop_types.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLineEdit;
class QVBoxLayout;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace teleop {

class TeleopSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit TeleopSettingsWidget(common::VisualizationManager* manager,
                                QWidget* parent = nullptr);

  void setConfig(const TeleopPanelConfig& config);
  TeleopPanelConfig config() const;

 signals:
  void configChanged();

 private slots:
  void emitConfigChanged();

 private:
  QWidget* makeButtonSection(const QString& title, TeleopButtonConfig* target);
  void rebuildButtonSections();

  common::VisualizationManager* manager_ = nullptr;
  TeleopPanelConfig config_;
  QLineEdit* title_edit_ = nullptr;
  QLineEdit* topic_edit_ = nullptr;
  QDoubleSpinBox* publish_rate_spin_ = nullptr;
  QCheckBox* stop_on_release_check_ = nullptr;
  QVBoxLayout* button_sections_layout_ = nullptr;
  TeleopButtonConfig* up_config_ = nullptr;
  TeleopButtonConfig* down_config_ = nullptr;
  TeleopButtonConfig* left_config_ = nullptr;
  TeleopButtonConfig* right_config_ = nullptr;
  TeleopButtonConfig* stop_config_ = nullptr;
};

}  // namespace teleop
}  // namespace autoviz

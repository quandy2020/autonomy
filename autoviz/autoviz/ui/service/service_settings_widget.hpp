/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/service/service_types.hpp"

class QCheckBox;
class QComboBox;
class QLineEdit;
class QPushButton;
class QSpinBox;

namespace autoviz {
namespace service_panel {

class ServiceSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit ServiceSettingsWidget(QWidget* parent = nullptr);

  ServiceCallPanelConfig config() const;
  void setConfig(const ServiceCallPanelConfig& config);

 signals:
  void configChanged();

 private slots:
  void pickButtonColor();

 private:
  void emitConfigChanged();

  ServiceCallPanelConfig config_;
  QLineEdit* title_edit_ = nullptr;
  QSpinBox* timeout_spin_ = nullptr;
  QComboBox* layout_combo_ = nullptr;
  QLineEdit* button_label_edit_ = nullptr;
  QLineEdit* button_tooltip_edit_ = nullptr;
  QPushButton* button_color_button_ = nullptr;
};

}  // namespace service_panel
}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/publish/publish_types.hpp"

class QLineEdit;
class QPushButton;

namespace autoviz {
namespace publish_panel {

class PublishSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit PublishSettingsWidget(QWidget* parent = nullptr);

  PublishPanelConfig config() const;
  void setConfig(const PublishPanelConfig& config);

 signals:
  void configChanged();

 private:
  void emitConfigChanged();
  void pickButtonColor();

  PublishPanelConfig config_;
  QLineEdit* title_edit_ = nullptr;
  QLineEdit* button_label_edit_ = nullptr;
  QLineEdit* button_tooltip_edit_ = nullptr;
  QPushButton* button_color_button_ = nullptr;
};

}  // namespace publish_panel
}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <QSet>

#include "autoviz/ui/log/log_types.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QVBoxLayout;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace log_panel {

class LogSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit LogSettingsWidget(common::VisualizationManager* manager,
                             QWidget* parent = nullptr);

  void setConfig(const LogPanelConfig& config);
  LogPanelConfig config() const;
  void setKnownNamespaces(const QSet<QString>& namespaces);

 signals:
  void configChanged();

 private slots:
  void emitConfigChanged();

 private:
  void rebuildNamespaceSection();

  common::VisualizationManager* manager_ = nullptr;
  LogPanelConfig config_;
  QSet<QString> known_namespaces_;
  QLineEdit* title_edit_ = nullptr;
  QLineEdit* topic_edit_ = nullptr;
  QComboBox* min_level_combo_ = nullptr;
  QDoubleSpinBox* font_size_spin_ = nullptr;
  QCheckBox* capture_glog_check_ = nullptr;
  QCheckBox* follow_playback_check_ = nullptr;
  QVBoxLayout* namespace_layout_ = nullptr;
};

}  // namespace log_panel
}  // namespace autoviz

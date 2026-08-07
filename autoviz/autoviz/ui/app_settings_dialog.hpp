/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include <QColor>
#include <QDialog>
#include <QHash>
#include <QKeySequence>
#include <QString>

#include "autoviz/ui/app_preferences.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;

namespace autoviz {

namespace common {
enum class TimeSyncMode;
class VisualizationManager;
}

class ShortcutsEditorWidget;

struct AppSettingsResult {
  std::string fixed_frame;
  std::string transformer_id;
  std::string view_controller;
  std::string render_backend;
  std::string background_color;
  int frame_rate = 30;
  bool show_grid = true;
  common::TimeSyncMode time_sync_mode{};
  bool time_paused = false;
  bool hide_left_dock = false;
  bool hide_right_dock = false;
  bool plot_settings_visible = true;
  bool start_maximized = true;
  QString language_code;
  QHash<QString, QKeySequence> shortcuts;
};

class AppSettingsDialog : public QDialog {
  Q_OBJECT

 public:
  explicit AppSettingsDialog(common::VisualizationManager* manager,
                             QWidget* parent = nullptr);

  AppSettingsResult resultValues() const;

 private:
  void populateFrameList();
  void syncBackgroundUiFromColor(const QColor& color);
  QColor backgroundColorFromUi() const;
  void pickBackgroundColor();

  common::VisualizationManager* manager_ = nullptr;
  QComboBox* language_combo_ = nullptr;
  ShortcutsEditorWidget* shortcuts_editor_ = nullptr;
  QComboBox* fixed_frame_combo_ = nullptr;
  QComboBox* transformer_combo_ = nullptr;
  QComboBox* view_controller_combo_ = nullptr;
  QComboBox* render_backend_combo_ = nullptr;
  QComboBox* time_sync_combo_ = nullptr;
  QSpinBox* frame_rate_spin_ = nullptr;
  QSlider* frame_rate_slider_ = nullptr;
  QPushButton* background_button_ = nullptr;
  QLabel* background_preview_ = nullptr;
  QSpinBox* background_r_spin_ = nullptr;
  QSpinBox* background_g_spin_ = nullptr;
  QSpinBox* background_b_spin_ = nullptr;
  QCheckBox* show_grid_check_ = nullptr;
  QCheckBox* time_paused_check_ = nullptr;
  QCheckBox* show_left_sidebar_check_ = nullptr;
  QCheckBox* show_right_sidebar_check_ = nullptr;
  QCheckBox* show_panel_settings_check_ = nullptr;
  QCheckBox* start_maximized_check_ = nullptr;
};

}  // namespace autoviz

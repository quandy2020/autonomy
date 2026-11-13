/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file settings_dialog.hpp
 * @brief Concat-style preferences dialog (sidebar rail + detail pages).
 *
 * Edits visualization manager options (fixed frame, backend, FPS, background)
 * and @ref AppUiPreferences (language, theme, accent, shortcuts, dock
 * visibility). Accepted values are returned via @ref AppSettingsResult.
 *
 * @see FrameSession::onAppSettings()
 * @see AppUiPreferences
 * @see ShortcutsEditorWidget
 */

#pragma once

#include <string>

#include <QColor>
#include <QDialog>
#include <QHash>
#include <QKeySequence>
#include <QString>

#include "autoviz/ui/app/preferences.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QListWidget;
class QPushButton;
class QSlider;
class QSpinBox;
class QStackedWidget;
class QWidget;

namespace autoviz {

namespace common {
enum class TimeSyncMode;
class VisualizationManager;
}

namespace widget {
class SegmentedControl;
}

class ShortcutsEditorWidget;

/**
 * @struct AppSettingsResult
 * @brief Snapshot of values accepted from @ref AppSettingsDialog.
 *
 * Applied by @ref FrameSession::applyUiPreferences() after the dialog is
 * accepted.
 */
struct AppSettingsResult {
  std::string fixed_frame;       /**< TF fixed frame name. */
  std::string transformer_id;    /**< TF transformer plugin id. */
  std::string render_backend;    /**< @c OpenGL / @c Ogre. */
  std::string background_color;  /**< Encoded viewport background. */
  int frame_rate = 30;           /**< Target render FPS. */
  common::TimeSyncMode time_sync_mode{}; /**< Time sync policy. */
  bool time_paused = false;      /**< Playback paused flag. */
  bool hide_left_dock = false;   /**< Hide left sidebar. */
  bool hide_right_dock = false;  /**< Hide right sidebar. */
  bool plot_settings_visible = true; /**< Plot settings pane visible. */
  bool start_maximized = true;   /**< Launch maximized. */
  QString language_code;         /**< UI locale (empty = system). */
  QString theme_mode; /**< @c system / @c light / @c dark / @c liquid. */
  QString accent_color; /**< Optional @c #RRGGBB accent override. */
  QHash<QString, QKeySequence> shortcuts; /**< Shortcut overrides. */
};

/**
 * @class AppSettingsDialog
 * @brief Concat-style preferences: gray rail + grouped detail pages.
 *
 * Appearance page uses a Dark / Light / Liquid segmented control and accent
 * color swatches (live-preview; Cancel restores the prior theme).
 *
 * @see FrameSession::onAppSettings()
 * @see ShortcutsEditorWidget
 */
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
  void showCategory(int index);
  void setThemeMode(const QString& theme_mode);
  void setAccentColor(const QString& accent_hex);
  void applyLiveThemePreview();
  void syncAccentSwatchSelection();

  common::VisualizationManager* manager_ = nullptr;
  QListWidget* sidebar_ = nullptr;
  QStackedWidget* pages_ = nullptr;
  QLabel* page_title_ = nullptr;

  QComboBox* language_combo_ = nullptr;
  widget::SegmentedControl* theme_segments_ = nullptr;
  QWidget* accent_host_ = nullptr;
  QLabel* accent_name_label_ = nullptr;
  QString theme_mode_ = QStringLiteral("light");
  QString accent_color_;
  QString preview_restore_theme_;
  QString preview_restore_accent_;

  ShortcutsEditorWidget* shortcuts_editor_ = nullptr;
  QComboBox* fixed_frame_combo_ = nullptr;
  QComboBox* transformer_combo_ = nullptr;
  QComboBox* render_backend_combo_ = nullptr;
  QComboBox* time_sync_combo_ = nullptr;
  QSpinBox* frame_rate_spin_ = nullptr;
  QSlider* frame_rate_slider_ = nullptr;
  QPushButton* background_button_ = nullptr;
  QLabel* background_preview_ = nullptr;
  QSpinBox* background_r_spin_ = nullptr;
  QSpinBox* background_g_spin_ = nullptr;
  QSpinBox* background_b_spin_ = nullptr;
  QCheckBox* time_paused_check_ = nullptr;
  QCheckBox* show_left_sidebar_check_ = nullptr;
  QCheckBox* show_right_sidebar_check_ = nullptr;
  QCheckBox* show_panel_settings_check_ = nullptr;
  QCheckBox* start_maximized_check_ = nullptr;
};

}  // namespace autoviz

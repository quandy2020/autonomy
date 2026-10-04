/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file settings_dialog.hpp
 * @brief macOS System Settings–style preferences dialog (sidebar + detail pages).
 *
 * Edits visualization manager options (fixed frame, backend, FPS, background)
 * and @ref AppUiPreferences (language, shortcuts, dock visibility, HUD-related
 * flags). Accepted values are returned via @ref AppSettingsResult.
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

namespace autoviz {

namespace common {
enum class TimeSyncMode;
class VisualizationManager;
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
  QHash<QString, QKeySequence> shortcuts; /**< Shortcut overrides. */
};

/**
 * @class AppSettingsDialog
 * @brief macOS System Settings–style preferences dialog (sidebar + detail pages).
 *
 * ## Pages
 *
 * Typical categories: General (language, start maximized), Shortcuts,
 * Visualization (frame, backend, FPS, background), Layout (sidebars, plot
 * settings). Exact set follows the sidebar list built in the constructor.
 *
 * @note Does not apply changes until the dialog is accepted; callers read
 *       @ref resultValues().
 *
 * @see FrameSession::onAppSettings()
 * @see ShortcutsEditorWidget
 */
class AppSettingsDialog : public QDialog {
  Q_OBJECT

 public:
  /**
   * @brief Builds the sidebar + stacked pages from @p manager state.
   *
   * @param manager Non-owning visualization manager (for TF frames, backend,
   *        time sync). Must outlive the dialog.
   * @param parent Parent widget (typically @ref VisualizationFrame).
   */
  explicit AppSettingsDialog(common::VisualizationManager* manager,
                             QWidget* parent = nullptr);

  /**
   * @brief Collects current widget values into an @ref AppSettingsResult.
   * @return Snapshot suitable for @ref FrameSession::applyUiPreferences().
   */
  AppSettingsResult resultValues() const;

 private:
  /**
   * @brief Fills the fixed-frame combo from the manager's TF tree.
   */
  void populateFrameList();

  /**
   * @brief Pushes @p color into the background button / preview / RGB spins.
   * @param color Viewport background color.
   */
  void syncBackgroundUiFromColor(const QColor& color);

  /**
   * @brief Builds a @c QColor from the RGB spin boxes (and button state).
   * @return Current background color from the UI.
   */
  QColor backgroundColorFromUi() const;

  /**
   * @brief Opens a color dialog and updates background widgets.
   */
  void pickBackgroundColor();

  /**
   * @brief Switches the stacked detail page and updates the page title.
   * @param index Sidebar row index.
   */
  void showCategory(int index);

  /** Non-owning visualization manager. */
  common::VisualizationManager* manager_ = nullptr;

  /** Category sidebar list. */
  QListWidget* sidebar_ = nullptr;

  /** Detail pages stack. */
  QStackedWidget* pages_ = nullptr;

  /** Title label above the active page. */
  QLabel* page_title_ = nullptr;

  /** UI language combo. */
  QComboBox* language_combo_ = nullptr;

  /** Shortcut table editor. */
  ShortcutsEditorWidget* shortcuts_editor_ = nullptr;

  /** Fixed TF frame combo. */
  QComboBox* fixed_frame_combo_ = nullptr;

  /** TF transformer combo. */
  QComboBox* transformer_combo_ = nullptr;

  /** Render backend combo. */
  QComboBox* render_backend_combo_ = nullptr;

  /** Time sync mode combo. */
  QComboBox* time_sync_combo_ = nullptr;

  /** Target FPS spin box. */
  QSpinBox* frame_rate_spin_ = nullptr;

  /** Target FPS slider (mirrors spin). */
  QSlider* frame_rate_slider_ = nullptr;

  /** Background color picker button. */
  QPushButton* background_button_ = nullptr;

  /** Background color preview swatch. */
  QLabel* background_preview_ = nullptr;

  /** Background red channel. */
  QSpinBox* background_r_spin_ = nullptr;

  /** Background green channel. */
  QSpinBox* background_g_spin_ = nullptr;

  /** Background blue channel. */
  QSpinBox* background_b_spin_ = nullptr;

  /** Pause time / playback checkbox. */
  QCheckBox* time_paused_check_ = nullptr;

  /** Show left sidebar checkbox. */
  QCheckBox* show_left_sidebar_check_ = nullptr;

  /** Show right sidebar checkbox. */
  QCheckBox* show_right_sidebar_check_ = nullptr;

  /** Show plot / panel settings checkbox. */
  QCheckBox* show_panel_settings_check_ = nullptr;

  /** Start maximized checkbox. */
  QCheckBox* start_maximized_check_ = nullptr;
};

}  // namespace autoviz

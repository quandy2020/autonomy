/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file panel.hpp
 * @brief Time panel — sim/wall clocks, pause/reset, and TF/time sync controls.
 *
 * Hosted as a bottom or side strip in VisualizationFrame. Reads clock state
 * from @ref common::VisualizationManager and exposes sync mode / source for
 * session persistence.
 *
 * @see ImportRecordDialog
 * @see common::VisualizationManager
 */

#pragma once

#include <QString>

#include <QWidget>

class QCheckBox;
class QComboBox;
class QHBoxLayout;
class QLineEdit;
class QPushButton;
class QLabel;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/**
 * @class TimePanel
 * @brief Compact time strip with experimental and legacy layouts.
 *
 * ## Layout (experimental)
 *
 * @code
 * ┌────────────────────────────────────────────────────────────┐
 * │ [Pause] [Reset]  Sync ▾  Source ▾   sim …  wall …  FPS …  │
 * └────────────────────────────────────────────────────────────┘
 * @endcode
 *
 * When @ref experimental() is false, a denser legacy row (@c old_widget_) is
 * shown instead of @c experimental_widget_. Toggling experimental emits
 * @ref layoutChanged() so the frame can relayout docks.
 *
 * ## Data flow
 *
 * - **Out:** Pause toggles manager pause; Reset emits @ref resetRequested();
 *   sync mode/source write into the manager and are readable via getters.
 * - **In:** @ref setFpsText() / @ref refreshTimes() / @ref syncAfterReset()
 *   update labels from the frame's render loop.
 *
 * @note Non-owning @c manager_; lifetime owned by VisualizationFrame.
 */
class TimePanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the panel and builds both experimental and legacy UIs.
   *
   * @param manager Non-owning visualization manager for clocks and sync.
   * @param parent Qt parent widget.
   */
  explicit TimePanel(common::VisualizationManager* manager,
                     QWidget* parent = nullptr);

  /**
   * @brief Whether the experimental time strip is active.
   *
   * @return @c true when @c experimental_widget_ is shown.
   */
  bool experimental() const;

  /**
   * @brief Current sync mode combo index / enum value.
   *
   * @return Sync mode as stored for session config.
   */
  int syncMode() const;

  /**
   * @brief Current sync source identifier (channel / clock name).
   *
   * @return Source string; may fall back to @c config_sync_source_.
   */
  QString syncSource() const;

  /**
   * @brief Shows the experimental or legacy layout.
   *
   * Emits @ref layoutChanged() when the visible chrome changes.
   *
   * @param enabled @c true for experimental strip.
   */
  void setExperimental(bool enabled);

  /**
   * @brief Programmatically selects a sync mode.
   *
   * @param mode Sync mode index matching the combo items.
   */
  void setSyncMode(int mode);

  /**
   * @brief Programmatically selects a sync source by name.
   *
   * @param source Source id; stored in @c config_sync_source_ if not yet in
   *        the combo (until @ref refreshSyncSources() runs).
   */
  void setSyncSource(const QString& source);

  /**
   * @brief Updates the FPS readout label.
   *
   * @param text Pre-formatted FPS string (e.g. @c "60 FPS").
   */
  void setFpsText(const QString& text);

  /**
   * @brief Uncheck Pause and refresh labels after a time reset.
   *
   * Called by VisualizationFrame after handling @ref resetRequested().
   */
  void syncAfterReset();

 signals:
  /**
   * @brief Emitted when the user clicks Reset; frame clears clocks / playback.
   */
  void resetRequested();

  /**
   * @brief Emitted when experimental/legacy layout visibility changes.
   *
   * Listeners should adjust dock sizes / chrome.
   */
  void layoutChanged();

 private slots:
  /**
   * @brief Pause button toggled: forwards pause state to the manager.
   *
   * @param checked @c true when paused.
   */
  void pauseToggled(bool checked);

  /**
   * @brief Sync mode combo activated.
   *
   * @param index Selected mode index.
   */
  void syncModeSelected(int index);

  /**
   * @brief Sync source combo activated.
   *
   * @param index Selected source index.
   */
  void syncSourceSelected(int index);

  /**
   * @brief Experimental checkbox toggled — switches visible layout.
   *
   * @param checked New experimental state.
   */
  void experimentalToggled(bool checked);

  /**
   * @brief Reset button: emits @ref resetRequested().
   */
  void onResetClicked();

  /**
   * @brief Periodic / on-demand refresh of sim and wall time labels.
   */
  void refreshTimes();

 private:
  /**
   * @brief Factory for a read-only time QLineEdit with theme styling.
   *
   * @return New line edit used as a time display field.
   */
  QLineEdit* makeTimeLabel();

  /**
   * @brief Formats @p time into @p label (seconds → display string).
   *
   * @param label Target read-only editor.
   * @param time Time value in seconds.
   */
  void fillTimeLabel(QLineEdit* label, double time);

  /**
   * @brief Repopulates the sync-source combo from the manager.
   *
   * Restores @c config_sync_source_ when present.
   */
  void refreshSyncSources();

  /**
   * @brief Emits @ref layoutChanged() after experimental visibility changes.
   */
  void notifyLayoutChanged();

  /** Non-owning; clocks, pause, and sync APIs. */
  common::VisualizationManager* manager_ = nullptr;

  /** Container for the experimental time strip. */
  QWidget* experimental_widget_ = nullptr;

  /** Container for the legacy time strip. */
  QWidget* old_widget_ = nullptr;

  /** Shared bottom row (FPS / common controls) when applicable. */
  QWidget* bottom_row_ = nullptr;

  /** Last sync source requested before the combo was populated. */
  QString config_sync_source_;

  /** Checkbox switching experimental vs legacy layout. */
  QCheckBox* experimental_cb_ = nullptr;

  /** Pause / resume toggle button. */
  QPushButton* pause_button_ = nullptr;

  /** Reset button emitting @ref resetRequested(). */
  QPushButton* reset_button_ = nullptr;

  /** Combo of available sync sources (channels / clocks). */
  QComboBox* sync_source_selector_ = nullptr;

  /** Combo of sync modes (off / exact / approx, …). */
  QComboBox* sync_mode_selector_ = nullptr;

  /** Sim time absolute readout. */
  QLineEdit* sim_time_label_ = nullptr;

  /** Sim time elapsed readout. */
  QLineEdit* sim_elapsed_label_ = nullptr;

  /** Wall time absolute readout. */
  QLineEdit* wall_time_label_ = nullptr;

  /** Wall time elapsed readout. */
  QLineEdit* wall_elapsed_label_ = nullptr;

  /** FPS text label updated via @ref setFpsText(). */
  QLabel* fps_label_ = nullptr;
};

}  // namespace autoviz

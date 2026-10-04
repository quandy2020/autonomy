/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_settings_widget.hpp
 * @brief Property editor for @ref PlotPanelConfig (axes, sync, series list).
 *
 * Embedded beside the chart or reparented into the shared inspector. Emits
 * @ref configChanged(), @ref addSeriesRequested(), and
 * @ref removeSeriesRequested().
 *
 * @see PlotPanel
 * @see PlotPanelConfig
 * @see plot_path_utils.hpp
 */

#pragma once

#include <QWidget>

#include "autoviz/ui/plot/plot_types.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QPushButton;
class QVBoxLayout;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace plot {

/**
 * @class PlotSettingsWidget
 * @brief Form UI for plot title, X-axis mode, sync, and per-series editors.
 *
 * ## Sections
 *
 * - General: title, X-axis mode, message-path mode, sync-with-other-plots
 * - Timestamp window / lock scales / legend values
 * - Collapsible series list with Value path editors and filters
 *
 * @note Does not own @ref common::VisualizationManager.
 */
class PlotSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Builds the settings form and wires change signals.
   *
   * @param manager Non-owning manager for channel / path suggestions.
   * @param parent Qt parent widget.
   */
  explicit PlotSettingsWidget(common::VisualizationManager* manager,
                              QWidget* parent = nullptr);

  /**
   * @brief Pushes config into all editors and rebuilds the series section.
   *
   * @param config Panel configuration to display.
   */
  void setConfig(const PlotPanelConfig& config);

  /**
   * @brief Reads the current form state into an @ref PlotPanelConfig.
   *
   * @return Config assembled from widget values.
   */
  PlotPanelConfig config() const;

  /**
   * @brief Default label for a new series at @p index (e.g. @c "Series 1").
   *
   * @param index Zero-based series index.
   * @return Localized default label.
   */
  static QString defaultSeriesLabel(int index);

  /**
   * @brief Repopulates channel-related editors from the manager.
   */
  void refreshChannelLists();

 signals:
  /** Emitted when any setting control changes. */
  void configChanged();

  /** Request appending a new series. */
  void addSeriesRequested();

  /**
   * @brief Request removing a series by index.
   *
   * @param index Series index in @ref PlotPanelConfig::series.
   */
  void removeSeriesRequested(int index);

 private slots:
  /** Emit @ref configChanged() from editor signals. */
  void emitConfigChanged();

  /** Add-series button clicked. */
  void onAddSeriesClicked();

  /** Remove button on a series editor clicked. */
  void onRemoveSeriesClicked();

 private:
  /**
   * @brief Wraps @p body in a collapsible titled section.
   *
   * @param title Section header text.
   * @param body Content widget.
   * @param expanded Initial expanded state.
   * @return Section host widget.
   */
  QWidget* makeCollapsibleSection(const QString& title, QWidget* body,
                                  bool expanded);

  /**
   * @brief Builds one series editor row for @p series at @p index.
   *
   * @param index Series index.
   * @param series Series configuration.
   * @return Editor widget.
   */
  QWidget* buildSeriesEditor(int index, const PlotSeriesConfig& series);

  /** Rebuild all series editor rows from @c config_.series. */
  void rebuildSeriesSection();

  /** Show/hide X-axis sub-panels for timestamp / index / message-path. */
  void updateAxisModeVisibility();

  /** Filter which series editors are visible via @c series_filter_combo_. */
  void applySeriesFilter();

  /**
   * @brief Refresh a series Value combo's editable text / suggestions.
   *
   * @param value_combo Value path combo for one series.
   */
  void refreshSeriesValueEdit(QComboBox* value_combo);

  /**
   * @brief Known channel names for path splitting / suggestions.
   *
   * @return Channel list.
   */
  QStringList knownChannels() const;

  /** Non-owning; channel discovery. */
  common::VisualizationManager* manager_ = nullptr;

  /** Working configuration mirrored by the form. */
  PlotPanelConfig config_;

  QLineEdit* title_edit_ = nullptr;  /**< Panel title. */
  QComboBox* x_axis_mode_combo_ = nullptr;  /**< X-axis mode. */
  QComboBox* message_path_mode_combo_ = nullptr;  /**< Current vs accumulated. */
  QComboBox* sync_plots_combo_ = nullptr;  /**< Sync with other plots. */
  QDoubleSpinBox* x_window_spin_ = nullptr;  /**< Timestamp window (seconds). */
  QWidget* x_axis_timestamp_body_ = nullptr;  /**< Timestamp-mode options. */
  QWidget* x_axis_index_body_ = nullptr;  /**< Index-mode options. */
  QWidget* x_axis_message_path_body_ = nullptr;  /**< Message-path-mode options. */
  QCheckBox* lock_axis_scales_check_ = nullptr;  /**< Lock axis scales. */
  QCheckBox* y_auto_scale_check_ = nullptr;  /**< Auto-fit Y axis. */
  QDoubleSpinBox* y_min_spin_ = nullptr;  /**< Fixed Y minimum. */
  QDoubleSpinBox* y_max_spin_ = nullptr;  /**< Fixed Y maximum. */
  QCheckBox* y_auto_scale_right_check_ = nullptr;  /**< Auto-fit right Y. */
  QDoubleSpinBox* y_min_right_spin_ = nullptr;  /**< Fixed right Y min. */
  QDoubleSpinBox* y_max_right_spin_ = nullptr;  /**< Fixed right Y max. */
  QCheckBox* show_grid_check_ = nullptr;  /**< Show chart grid. */
  QCheckBox* show_reference_y_check_ = nullptr;  /**< Show reference line. */
  QDoubleSpinBox* reference_y_spin_ = nullptr;  /**< Reference line Y. */
  QCheckBox* show_legend_values_check_ = nullptr;  /**< Legend value column. */
  QVBoxLayout* series_list_layout_ = nullptr;  /**< Dynamic series editors. */
  QWidget* series_container_ = nullptr;  /**< Series list host. */
  QComboBox* series_filter_combo_ = nullptr;  /**< Filter series list. */
  QPushButton* add_series_button_ = nullptr;  /**< Add series. */
};

}  // namespace plot
}  // namespace autoviz

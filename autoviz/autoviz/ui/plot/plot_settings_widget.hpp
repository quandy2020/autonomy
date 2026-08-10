/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

class PlotSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit PlotSettingsWidget(common::VisualizationManager* manager,
                              QWidget* parent = nullptr);

  void setConfig(const PlotPanelConfig& config);
  PlotPanelConfig config() const;
  static QString defaultSeriesLabel(int index);
  void refreshChannelLists();

 signals:
  void configChanged();
  void addSeriesRequested();
  void removeSeriesRequested(int index);

 private slots:
  void emitConfigChanged();
  void onAddSeriesClicked();
  void onRemoveSeriesClicked();

 private:
  QWidget* makeCollapsibleSection(const QString& title, QWidget* body,
                                  bool expanded);
  QWidget* buildSeriesEditor(int index, const PlotSeriesConfig& series);
  void rebuildSeriesSection();
  void updateAxisModeVisibility();
  void applySeriesFilter();
  void refreshSeriesValueEdit(QComboBox* value_combo);
  QStringList knownChannels() const;

  common::VisualizationManager* manager_ = nullptr;
  PlotPanelConfig config_;
  QLineEdit* title_edit_ = nullptr;
  QComboBox* x_axis_mode_combo_ = nullptr;
  QComboBox* message_path_mode_combo_ = nullptr;
  QComboBox* sync_plots_combo_ = nullptr;
  QDoubleSpinBox* x_window_spin_ = nullptr;
  QWidget* x_axis_timestamp_body_ = nullptr;
  QWidget* x_axis_index_body_ = nullptr;
  QWidget* x_axis_message_path_body_ = nullptr;
  QCheckBox* lock_axis_scales_check_ = nullptr;
  QCheckBox* show_legend_values_check_ = nullptr;
  QVBoxLayout* series_list_layout_ = nullptr;
  QWidget* series_container_ = nullptr;
  QComboBox* series_filter_combo_ = nullptr;
  QPushButton* add_series_button_ = nullptr;
};

}  // namespace plot
}  // namespace autoviz

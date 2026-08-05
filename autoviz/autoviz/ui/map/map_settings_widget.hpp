/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/map/map_types.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace map {

class MapSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit MapSettingsWidget(common::VisualizationManager* manager,
                             QWidget* parent = nullptr);

  MapPanelConfig config();
  void setConfig(const MapPanelConfig& config);
  void refreshChannels();

 signals:
  void configChanged();

 private slots:
  void onAddTopicLayer();
  void onRemoveTopicLayer();
  void onTopicSelectionChanged();
  void onAddOverlayLayer();
  void onRemoveOverlayLayer();
  void onOverlaySelectionChanged();
  void emitConfigChanged();

 private:
  void rebuildTopicList();
  void rebuildOverlayList();
  void loadTopicEditor(const MapTopicLayerConfig& layer);
  void loadOverlayEditor(const MapOverlayLayerConfig& layer);
  MapTopicLayerConfig readTopicEditor() const;
  MapOverlayLayerConfig readOverlayEditor() const;
  void saveCurrentTopicEditor();
  void saveCurrentOverlayEditor();
  QColor defaultColorForIndex(int index) const;

  common::VisualizationManager* manager_ = nullptr;
  MapPanelConfig config_;
  int selected_topic_index_ = -1;
  int selected_overlay_index_ = -1;

  QLineEdit* title_edit_ = nullptr;
  QComboBox* base_layer_combo_ = nullptr;
  QLineEdit* custom_tile_url_edit_ = nullptr;
  QComboBox* follow_channel_combo_ = nullptr;
  QDoubleSpinBox* center_lat_spin_ = nullptr;
  QDoubleSpinBox* center_lon_spin_ = nullptr;
  QDoubleSpinBox* zoom_spin_ = nullptr;

  QListWidget* topic_list_ = nullptr;
  QComboBox* topic_channel_combo_ = nullptr;
  QComboBox* topic_style_combo_ = nullptr;
  QCheckBox* topic_show_heading_check_ = nullptr;
  QCheckBox* topic_show_velocity_check_ = nullptr;
  QDoubleSpinBox* topic_point_size_spin_ = nullptr;
  QComboBox* topic_time_range_combo_ = nullptr;
  QDoubleSpinBox* topic_time_seconds_spin_ = nullptr;
  QDoubleSpinBox* topic_opacity_spin_ = nullptr;
  QPushButton* topic_color_button_ = nullptr;
  QCheckBox* topic_enabled_check_ = nullptr;
  QWidget* topic_editor_ = nullptr;

  QListWidget* overlay_list_ = nullptr;
  QLineEdit* overlay_name_edit_ = nullptr;
  QLineEdit* overlay_url_edit_ = nullptr;
  QDoubleSpinBox* overlay_opacity_spin_ = nullptr;
  QCheckBox* overlay_enabled_check_ = nullptr;
  QWidget* overlay_editor_ = nullptr;
};

}  // namespace map
}  // namespace autoviz

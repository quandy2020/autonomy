/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_settings_widget.hpp
 * @brief Property editor for @ref MapPanelConfig (basemap, topics, overlays).
 *
 * Provides list+detail editors for topic layers and tile overlay layers.
 * Emits @ref configChanged() on edits for @ref MapPanel to apply.
 *
 * @see MapPanel
 * @see MapPanelConfig
 */

#pragma once

#include <QWidget>

#include "autoviz/ui/map/map_types.hpp"

class QCheckBox;
class QComboBox;
class QGroupBox;
class QDoubleSpinBox;
class QLineEdit;
class QListWidget;
class QPushButton;
class QLabel;
class QSpinBox;
class QTableWidget;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace map {

/**
 * @class MapSettingsWidget
 * @brief Form UI for map title, camera, basemap, topic layers, and overlays.
 *
 * ## Sections
 *
 * - Title, base layer, custom tile URL, follow channel, center/zoom
 * - Topic layer list + style editor (point style, time range, color, …)
 * - Overlay tile layer list + URL / opacity editor
 *
 * @note Does not own @ref common::VisualizationManager.
 */
class MapSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Builds the settings form and wires editor signals.
   *
   * @param manager Non-owning manager for channel lists.
   * @param parent Qt parent widget.
   */
  explicit MapSettingsWidget(common::VisualizationManager* manager,
                             QWidget* parent = nullptr);

  /**
   * @brief Reads the current form (flushing active editors) into config.
   *
   * @return Assembled @ref MapPanelConfig.
   * @note Non-const because it may save the current topic/overlay editor
   *       back into @c config_ before returning.
   */
  MapPanelConfig config();

  /**
   * @brief Pushes config into all editors and rebuilds lists.
   *
   * @param config Panel configuration to display.
   */
  void setConfig(const MapPanelConfig& config);

  /**
   * @brief Shows the selected feature or plan vertex in the attribute group.
   *
   * @param info Inspector payload. An invalid info clears the table.
   */
  void setSelection(const MapSelectionInfo& info);

  /**
   * @brief Repopulates follow / topic channel combos from the manager.
   */
  void refreshChannels();

 signals:
  /**
   * @brief Emitted when any setting control changes.
   */
  void configChanged();

  /**
   * @brief Asks the viewport to cache tiles for the current view.
   *
   * @param min_zoom Lowest tile zoom.
   * @param max_zoom Highest tile zoom.
   */
  void offlineDownloadRequested(int min_zoom, int max_zoom);

  /**
   * @brief Latitude or longitude of the selected plan vertex was edited.
   *
   * @param kind 0 waypoint, 1 geofence, 2 rally.
   * @param index Vertex index.
   * @param latitude New latitude.
   * @param longitude New longitude.
   */
  void planVertexEdited(int kind, int index, double latitude, double longitude);

  /**
   * @brief Asks the viewport to delete the selected plan vertex.
   *
   * @param kind 0 waypoint, 1 geofence, 2 rally.
   * @param index Vertex index.
   */
  void planVertexRemoved(int kind, int index);

 private slots:
  /** Append a new topic layer with a default color. */
  void onAddTopicLayer();

  /** Remove the selected topic layer. */
  void onRemoveTopicLayer();

  /** Load the selected topic layer into the detail editor. */
  void onTopicSelectionChanged();

  /** Append a new overlay tile layer. */
  void onAddOverlayLayer();

  /** Remove the selected overlay layer. */
  void onRemoveOverlayLayer();

  /** Load the selected overlay into the detail editor. */
  void onOverlaySelectionChanged();

  /** Emit @ref configChanged() from editor signals. */
  void emitConfigChanged();

 private:
  /** Rebuild the topic list widget from @c config_.topic_layers. */
  void rebuildTopicList();

  /** Rebuilds the GeoJSON file list from @c config_. */
  void rebuildGeoJsonList();

  /** Rebuild the overlay list widget from @c config_.overlay_layers. */
  void rebuildOverlayList();

  /**
   * @brief Populate topic detail editors from @p layer.
   *
   * @param layer Topic layer to display.
   */
  void loadTopicEditor(const MapTopicLayerConfig& layer);

  /**
   * @brief Populate overlay detail editors from @p layer.
   *
   * @param layer Overlay layer to display.
   */
  void loadOverlayEditor(const MapOverlayLayerConfig& layer);

  /**
   * @brief Read topic detail editors into a config struct.
   *
   * @return Topic layer config from the form.
   */
  MapTopicLayerConfig readTopicEditor() const;

  /**
   * @brief Read overlay detail editors into a config struct.
   *
   * @return Overlay layer config from the form.
   */
  MapOverlayLayerConfig readOverlayEditor() const;

  /** Write the current topic editor back into @c config_ at the selection. */
  void saveCurrentTopicEditor();

  /** Write the current overlay editor back into @c config_ at the selection. */
  void saveCurrentOverlayEditor();

  /**
   * @brief Picks a distinct default color for a new topic layer.
   *
   * @param index Layer index in the list.
   * @return Suggested @c QColor.
   */
  QColor defaultColorForIndex(int index) const;

  /** Non-owning; channel discovery. */
  common::VisualizationManager* manager_ = nullptr;

  /** Working configuration mirrored by the form. */
  MapPanelConfig config_;

  /** Index of the topic layer loaded in the detail editor (−1 = none). */
  int selected_topic_index_ = -1;

  /** Index of the overlay layer loaded in the detail editor (−1 = none). */
  int selected_overlay_index_ = -1;

  /** Selection mirrored by the attribute inspector. */
  MapSelectionInfo selection_;

  QGroupBox* attribute_group_ = nullptr;         /**< Attribute inspector. */
  QLabel* attribute_hint_ = nullptr;             /**< Empty-selection hint. */
  QWidget* attribute_body_ = nullptr;            /**< Fields shown when something is selected. */
  QLabel* attribute_kind_label_ = nullptr;       /**< Feature kind. */
  QLabel* attribute_layer_label_ = nullptr;      /**< Layer or plan name. */
  QDoubleSpinBox* attribute_lat_spin_ = nullptr; /**< Selected latitude. */
  QDoubleSpinBox* attribute_lon_spin_ = nullptr; /**< Selected longitude. */
  QTableWidget* attribute_table_ = nullptr;      /**< Property rows. */
  QPushButton* attribute_remove_button_ = nullptr; /**< Delete the selected plan vertex. */

  QLineEdit* title_edit_ = nullptr;              /**< Panel title. */
  QComboBox* base_layer_combo_ = nullptr;        /**< Basemap preset. */
  QLineEdit* custom_tile_url_edit_ = nullptr;    /**< Custom tile URL template. */
  QComboBox* follow_channel_combo_ = nullptr;    /**< Follow-camera channel. */
  QComboBox* gcs_channel_combo_ = nullptr;       /**< Ground-station channel. */
  QComboBox* distance_unit_combo_ = nullptr;     /**< Scale-bar units. */
  QDoubleSpinBox* center_lat_spin_ = nullptr;    /**< Center latitude. */
  QDoubleSpinBox* center_lon_spin_ = nullptr;    /**< Center longitude. */
  QDoubleSpinBox* zoom_spin_ = nullptr;          /**< Map zoom. */
  QComboBox* edit_tool_combo_ = nullptr;         /**< Click tool. */
  QDoubleSpinBox* survey_spacing_spin_ = nullptr; /**< Survey spacing. */
  QDoubleSpinBox* corridor_width_spin_ = nullptr; /**< Corridor width. */
  QDoubleSpinBox* structure_radius_spin_ = nullptr; /**< Structure-scan radius. */
  QSpinBox* offline_min_zoom_spin_ = nullptr;    /**< Offline pack min zoom. */
  QSpinBox* offline_max_zoom_spin_ = nullptr;    /**< Offline pack max zoom. */

  QListWidget* topic_list_ = nullptr;            /**< Topic layer list. */
  QComboBox* topic_channel_combo_ = nullptr;     /**< Topic channel picker. */
  QComboBox* topic_style_combo_ = nullptr;       /**< Point style. */
  QCheckBox* topic_show_heading_check_ = nullptr; /**< Draw heading. */
  QCheckBox* topic_show_velocity_check_ = nullptr; /**< Draw velocity. */
  QDoubleSpinBox* topic_point_size_spin_ = nullptr; /**< Point size. */
  QComboBox* topic_time_range_combo_ = nullptr;  /**< Time-range mode. */
  QDoubleSpinBox* topic_time_seconds_spin_ = nullptr; /**< Last-N-seconds value. */
  QDoubleSpinBox* topic_opacity_spin_ = nullptr; /**< Layer opacity. */
  QPushButton* topic_color_button_ = nullptr;    /**< Color picker button. */
  QCheckBox* topic_enabled_check_ = nullptr;     /**< Layer enabled. */
  QWidget* topic_editor_ = nullptr;              /**< Topic detail host. */

  QListWidget* overlay_list_ = nullptr;          /**< Overlay layer list. */
  QLineEdit* overlay_name_edit_ = nullptr;       /**< Overlay display name. */
  QLineEdit* overlay_url_edit_ = nullptr;        /**< Overlay tile URL template. */
  QDoubleSpinBox* overlay_opacity_spin_ = nullptr; /**< Overlay opacity. */
  QCheckBox* overlay_enabled_check_ = nullptr;   /**< Overlay enabled. */
  QWidget* overlay_editor_ = nullptr;            /**< Overlay detail host. */

  QListWidget* geojson_list_ = nullptr;          /**< GeoJSON file list. */
};

}  // namespace map
}  // namespace autoviz

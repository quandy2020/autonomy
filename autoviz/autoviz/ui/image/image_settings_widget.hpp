/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_settings_widget.hpp
 * @brief Property editor for @ref ImagePanelConfig (channels, transforms, overlays).
 *
 * Embedded beside the image view or reparented into the shared inspector.
 * Edits emit @ref configChanged() so @ref ImagePanel can apply and persist.
 *
 * @see ImagePanel
 * @see ImagePanelConfig
 */

#pragma once

#include <QWidget>

#include "autoviz/ui/image/image_types.hpp"

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

namespace image {

/**
 * @class ImageSettingsWidget
 * @brief Form UI for editing Image panel title, sources, and display options.
 *
 * ## Sections
 *
 * - Title / main image channel / calibration channel
 * - Sync, undistort, flip, rotation, color mode
 * - Overlay list (add/remove), annotation and marker channel lists
 * - Background, label scale, click/hover publish topics
 *
 * @note Does not own @ref common::VisualizationManager.
 */
class ImageSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Builds the settings form and wires change signals.
   *
   * @param manager Non-owning manager for channel list population.
   * @param parent Qt parent widget.
   */
  explicit ImageSettingsWidget(common::VisualizationManager* manager,
                               QWidget* parent = nullptr);

  /**
   * @brief Pushes a config into all editors (suppresses emit loops as needed).
   *
   * @param config Panel configuration to display.
   * @see config()
   */
  void setConfig(const ImagePanelConfig& config);

  /**
   * @brief Reads the current form state into an @ref ImagePanelConfig.
   *
   * @return Config assembled from widget values.
   */
  ImagePanelConfig config() const;

  /**
   * @brief Repopulates channel combos from the visualization manager.
   */
  void refreshChannelLists();

 protected:
  /**
   * @brief Refreshes channel lists when the settings pane becomes visible.
   *
   * @param event Show event.
   */
  void showEvent(QShowEvent* event) override;

 signals:
  /**
   * @brief Emitted when any setting control changes.
   */
  void configChanged();

  /**
   * @brief Request appending a new overlay row.
   */
  void addOverlayRequested();

  /**
   * @brief Request removing an overlay by index.
   *
   * @param index Overlay index in @ref ImagePanelConfig::overlays.
   */
  void removeOverlayRequested(int index);

  /**
   * @brief Request moving an overlay by @p delta positions (-1 up / +1 down).
   *
   * @param index Overlay index.
   * @param delta Relative move.
   */
  void moveOverlayRequested(int index, int delta);

 private slots:
  /**
   * @brief Slot that emits @ref configChanged() from editor signals.
   */
  void emitConfigChanged();

 private:
  /** Rebuild the dynamic overlay editor rows. */
  void rebuildOverlaySection();

  /** Rebuild the annotation channel editor rows. */
  void rebuildAnnotationSection();

  /** Rebuild the marker channel editor rows. */
  void rebuildMarkerSection();

  /** Rebuild the PointCloud2 channel editor rows. */
  void rebuildPointCloudSection();

  /** Refresh items in the main image channel combo. */
  void refreshImageChannelItems();

  /**
   * @brief Discovered image-compatible channel names.
   *
   * @return Channel list for the main / overlay pickers.
   */
  QStringList imageChannels() const;

  /**
   * @brief Discovered CameraInfo / calibration channel names.
   *
   * @return Calibration channel list.
   */
  QStringList calibrationChannels() const;

  /**
   * @brief Discovered annotation-compatible channel names.
   *
   * @return Annotation channel list.
   */
  QStringList annotationChannels() const;

  /**
   * @brief Discovered marker-compatible channel names.
   *
   * @return Marker channel list.
   */
  QStringList markerChannels() const;

  /**
   * @brief Discovered PointCloud2 channel names.
   *
   * @return Point cloud channel list.
   */
  QStringList pointCloudChannels() const;

  /** Non-owning; provides discovered channels. */
  common::VisualizationManager* manager_ = nullptr;

  /** Working copy mirrored into / from editors. */
  ImagePanelConfig config_;

  QLineEdit* title_edit_ = nullptr;           /**< Panel title. */
  QComboBox* channel_combo_ = nullptr;        /**< Main image channel. */
  QComboBox* calibration_combo_ = nullptr;    /**< CameraInfo channel. */
  QCheckBox* strict_sync_check_ = nullptr;    /**< Strict time sync with overlays. */
  QCheckBox* undistort_check_ = nullptr;      /**< Enable lens undistortion. */
  QCheckBox* flip_h_check_ = nullptr;         /**< Flip horizontal. */
  QCheckBox* flip_v_check_ = nullptr;         /**< Flip vertical. */
  QComboBox* rotation_combo_ = nullptr;       /**< Discrete rotation. */
  QComboBox* color_mode_combo_ = nullptr;     /**< False-color mode. */
  QDoubleSpinBox* color_min_spin_ = nullptr;  /**< Colormap min. */
  QDoubleSpinBox* color_max_spin_ = nullptr;  /**< Colormap max. */
  QDoubleSpinBox* label_scale_spin_ = nullptr; /**< Annotation label scale. */
  QLineEdit* background_edit_ = nullptr;      /**< Background color string. */
  QLineEdit* click_topic_edit_ = nullptr;     /**< Pixel-click publish channel. */
  QLineEdit* hover_topic_edit_ = nullptr;     /**< Pixel-hover publish channel. */
  QVBoxLayout* overlay_list_layout_ = nullptr;     /**< Dynamic overlay rows. */
  QVBoxLayout* annotation_list_layout_ = nullptr;  /**< Annotation channel rows. */
  QVBoxLayout* marker_list_layout_ = nullptr;      /**< Marker channel rows. */
  QVBoxLayout* point_cloud_list_layout_ = nullptr; /**< PointCloud2 channel rows. */
  QPushButton* add_overlay_button_ = nullptr;      /**< Add overlay. */
};

}  // namespace image
}  // namespace autoviz

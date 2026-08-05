/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

class ImageSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit ImageSettingsWidget(common::VisualizationManager* manager,
                               QWidget* parent = nullptr);

  void setConfig(const ImagePanelConfig& config);
  ImagePanelConfig config() const;
  void refreshChannelLists();

 signals:
  void configChanged();
  void addOverlayRequested();
  void removeOverlayRequested(int index);

 private slots:
  void emitConfigChanged();

 private:
  QWidget* makeCollapsibleSection(const QString& title, QWidget* body,
                                  bool expanded);
  void rebuildOverlaySection();
  void rebuildAnnotationSection();
  void rebuildMarkerSection();
  QStringList imageChannels() const;
  QStringList calibrationChannels() const;
  QStringList annotationChannels() const;
  QStringList markerChannels() const;

  common::VisualizationManager* manager_ = nullptr;
  ImagePanelConfig config_;
  QLineEdit* title_edit_ = nullptr;
  QComboBox* topic_combo_ = nullptr;
  QComboBox* calibration_combo_ = nullptr;
  QCheckBox* strict_sync_check_ = nullptr;
  QCheckBox* undistort_check_ = nullptr;
  QCheckBox* flip_h_check_ = nullptr;
  QCheckBox* flip_v_check_ = nullptr;
  QComboBox* rotation_combo_ = nullptr;
  QComboBox* color_mode_combo_ = nullptr;
  QDoubleSpinBox* color_min_spin_ = nullptr;
  QDoubleSpinBox* color_max_spin_ = nullptr;
  QDoubleSpinBox* label_scale_spin_ = nullptr;
  QLineEdit* background_edit_ = nullptr;
  QLineEdit* click_topic_edit_ = nullptr;
  QLineEdit* hover_topic_edit_ = nullptr;
  QVBoxLayout* overlay_list_layout_ = nullptr;
  QVBoxLayout* annotation_list_layout_ = nullptr;
  QVBoxLayout* marker_list_layout_ = nullptr;
  QPushButton* add_overlay_button_ = nullptr;
};

}  // namespace image
}  // namespace autoviz

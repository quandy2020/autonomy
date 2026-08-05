/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <memory>
#include <string>
#include <vector>

#include <QPointer>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>

#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/image/image_annotation_parser.hpp"
#include "autoviz/ui/image/image_calibration_utils.hpp"
#include "autoviz/ui/image/image_marker_projection.hpp"
#include "autoviz/ui/image/image_types.hpp"
#include "autoviz/ui/image/image_video_decoder.hpp"

class QDragEnterEvent;
class QFocusEvent;
class QImage;
class QScrollArea;
class QTimer;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace image {
class ImageSettingsWidget;
class ImageViewWidget;
}

namespace image {

class ImagePanel : public QWidget {
  Q_OBJECT

 public:
  explicit ImagePanel(common::VisualizationManager* manager,
                      QWidget* parent = nullptr);
  ~ImagePanel() override;

  void installTitleBarTools(PanelDockWidget* dock);

  ImagePanelConfig config() const;
  void setConfig(const ImagePanelConfig& config);
  void cloneConfigFrom(const ImagePanelConfig& config);
  void applySettings(const ImagePanelConfig& config);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  void setExpandButtonChecked(bool checked);
  void refreshSettingsChannels();
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();

 signals:
  void configChanged();
  void activated();
  void settingsToggled(bool visible);
  void panelSplitRequested(Qt::Orientation orientation);
  void panelRemoveRequested();
  void panelExpandRequested();
  void panelChangeRequested(const QString& object_name);

 protected:
  void focusInEvent(QFocusEvent* event) override;
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dropEvent(QDropEvent* event) override;

 private slots:
  void onToggleSettings(bool visible);
  void onFrameTick();

 private:
  struct OverlayRuntime {
    ImageOverlayConfig config;
    integration::ChannelReaderRegistry::SubscriptionId subscription_id = 0;
    QImage image;
    qint64 timestamp_ns = 0;
  };

  struct AnnotationRuntime {
    QString channel;
    integration::ChannelReaderRegistry::SubscriptionId subscription_id = 0;
    ImageAnnotationLayer layer;
    qint64 timestamp_ns = 0;
  };

  struct MarkerRuntime {
    QString channel;
    integration::ChannelReaderRegistry::SubscriptionId subscription_id = 0;
    ImageAnnotationLayer layer;
    qint64 timestamp_ns = 0;
  };

  void resubscribeAll();
  void unsubscribeMain();
  void subscribeMain();
  void unsubscribeOverlays();
  void subscribeOverlays();
  void unsubscribeAnnotations();
  void subscribeAnnotations();
  void unsubscribeCalibration();
  void subscribeCalibration();
  void unsubscribeMarkers();
  void subscribeMarkers();
  void handleMainPayload(const std::string& payload);
  void handleOverlayPayload(int index, const std::string& payload);
  void handleAnnotationPayload(int index, const std::string& payload);
  void handleCalibrationPayload(const std::string& payload);
  void handleMarkerPayload(int index, const std::string& payload);
  QImage decodePayload(const std::string& message_type,
                       const std::string& payload);
  std::string messageTypeForChannel(const std::string& channel) const;
  void updateFixedToOptical();
  void mergeAnnotationLayer(ImageAnnotationLayer* destination,
                            const ImageAnnotationLayer& source) const;
  void updateRenderedFrame();
  void publishPixel(const QString& channel, int x, int y) const;
  void syncSettingsWidgetFromConfig();
  void syncSettingsToolState();
  void applyConfigToUi();

  common::VisualizationManager* manager_ = nullptr;
  ImagePanelConfig config_;
  ImageViewWidget* view_ = nullptr;
  ImageSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QPointer<QToolButton> settings_button_;
  QPointer<QToolButton> expand_button_;
  integration::ChannelReaderRegistry::SubscriptionId main_subscription_id_ = 0;
  integration::ChannelReaderRegistry::SubscriptionId calibration_subscription_id_ = 0;
  QImage base_image_;
  qint64 base_timestamp_ns_ = 0;
  CameraIntrinsics camera_intrinsics_;
  automsgs::msgs::sensor_msgs::CameraInfo camera_info_;
  bool have_camera_info_ = false;
  QMatrix4x4 fixed_to_optical_;
  bool have_fixed_to_optical_ = false;
  std::vector<OverlayRuntime> overlay_runtime_;
  std::vector<AnnotationRuntime> annotation_runtime_;
  std::vector<MarkerRuntime> marker_runtime_;
  VideoStreamDecoder video_decoder_;
  QTimer* frame_timer_ = nullptr;
};

}  // namespace image
}  // namespace autoviz

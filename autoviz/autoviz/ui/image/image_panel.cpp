/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_panel.hpp"

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFocusEvent>
#include <QFrame>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QMimeData>
#include <QQuaternion>
#include <QScrollArea>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include "autolink/message/raw_message.hpp"
#include "automsgs/msgs/geometry_msgs/point.pb.h"
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/compressed_image.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/image_utils.hpp"
#include "autoviz/integration/channel_payload.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/message_queue.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/image/image_annotation_parser.hpp"
#include "autoviz/ui/image/image_calibration_utils.hpp"
#include "autoviz/ui/image/image_marker_projection.hpp"
#include "autoviz/ui/image/image_processing.hpp"
#include "autoviz/ui/image/image_settings_widget.hpp"
#include "autoviz/ui/image/image_video_decoder.hpp"
#include "autoviz/ui/image/image_view_widget.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"

namespace autoviz {
namespace image {
namespace {


qint64 HeaderTimestampNs(const automsgs::msgs::std_msgs::Header& header) {
  return static_cast<qint64>(header.stamp().sec()) * 1000000000LL +
         static_cast<qint64>(header.stamp().nanosec());
}

QMatrix4x4 TransformToMatrix(
    const automsgs::msgs::geometry_msgs::Transform& transform) {
  QMatrix4x4 matrix;
  matrix.setToIdentity();
  matrix.translate(static_cast<float>(transform.translation().x()),
                   static_cast<float>(transform.translation().y()),
                   static_cast<float>(transform.translation().z()));
  matrix.rotate(
      QQuaternion(static_cast<float>(transform.rotation().w()),
                  static_cast<float>(transform.rotation().x()),
                  static_cast<float>(transform.rotation().y()),
                  static_cast<float>(transform.rotation().z())));
  return matrix;
}

QImage DecodeCompressedVideoJson(const std::string& payload,
                                 VideoStreamDecoder* decoder) {
  if (decoder == nullptr || payload.empty()) {
    return {};
  }
  const QJsonDocument document =
      QJsonDocument::fromJson(QByteArray::fromStdString(payload));
  if (!document.isObject()) {
    return {};
  }
  const QJsonObject root = document.object();
  const QString format = root.value(QStringLiteral("format")).toString();
  QByteArray data;
  const QJsonValue data_value = root.value(QStringLiteral("data"));
  if (data_value.isString()) {
    data = QByteArray::fromBase64(data_value.toString().toUtf8());
  } else if (data_value.isArray()) {
    data.reserve(data_value.toArray().size());
    for (const QJsonValue& byte_value : data_value.toArray()) {
      data.append(static_cast<char>(byte_value.toInt()));
    }
  }
  if (data.isEmpty()) {
    return {};
  }
  return decoder->decodePacket(
      format, reinterpret_cast<const std::byte*>(data.constData()),
      static_cast<std::size_t>(data.size()));
}

}  // namespace

ImagePanel::ImagePanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  settings_container_ = new QWidget(this);
  settings_container_->hide();
  auto* settings_layout = new QVBoxLayout(settings_container_);
  settings_layout->setContentsMargins(0, 0, 0, 0);
  settings_scroll_ = new QScrollArea(settings_container_);
  settings_scroll_->setWidgetResizable(true);
  settings_scroll_->setFrameShape(QFrame::NoFrame);
  settings_scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  settings_widget_ = new ImageSettingsWidget(manager_, settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
  settings_layout->addWidget(settings_scroll_);

  view_ = new ImageViewWidget(this);
  root->addWidget(view_, 1);

  connect(settings_widget_, &ImageSettingsWidget::configChanged, this, [this]() {
    applySettings(settings_widget_->config());
  });
  connect(settings_widget_, &ImageSettingsWidget::addOverlayRequested, this, [this]() {
    ImagePanelConfig updated = config_;
    ImageOverlayConfig overlay;
    overlay.opacity = 0.5;
    updated.overlays.push_back(overlay);
    setConfig(updated);
    emit configChanged();
  });
  connect(settings_widget_, &ImageSettingsWidget::removeOverlayRequested, this,
          [this](int index) {
            ImagePanelConfig updated = config_;
            if (index >= 0 && index < updated.overlays.size()) {
              updated.overlays.removeAt(index);
              setConfig(updated);
              emit configChanged();
            }
          });

  connect(view_, &ImageViewWidget::pixelClicked, this,
          [this](int x, int y) { publishPixel(config_.click_publish_channel, x, y); });
  connect(view_, &ImageViewWidget::pixelHovered, this,
          [this](int x, int y) { publishPixel(config_.hover_publish_channel, x, y); });

  frame_timer_ = new QTimer(this);
  frame_timer_->setTimerType(Qt::PreciseTimer);
  connect(frame_timer_, &QTimer::timeout, this, &ImagePanel::onFrameTick);
  frame_timer_->start(33);

  syncSettingsWidgetFromConfig();
  applyConfigToUi();
  // Default image_channel is non-empty; subscribe immediately so tutorials
  // show frames without requiring a Settings round-trip.
  resubscribeAll();
}

ImagePanel::~ImagePanel() {
  unsubscribeMain();
  unsubscribeOverlays();
  unsubscribeAnnotations();
  unsubscribeCalibration();
  unsubscribeMarkers();
}

void ImagePanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("ImageDock");
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.split = [this](Qt::Orientation orientation) {
    emit panelSplitRequested(orientation);
  };
  callbacks.expand = [this]() { emit panelExpandRequested(); };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };

  PanelTitleBarOptions options;
  options.show_reset = true;
  options.on_reset = [this]() { if (view_ != nullptr) { view_->resetView(); } };
  options.show_settings = true;
  options.settings_checked = config_.settings_visible;
  options.on_settings_toggled = [this](bool visible) { onToggleSettings(visible); };
  options.on_expand = [this]() { emit panelExpandRequested(); };

  const PanelTitleBarTools tools =
      CreateRvizPanelTitleBarTools(dock, callbacks, options);
  settings_button_ = tools.settings_button;
  expand_button_ = tools.expand_button;
  dock->setTitleBarTools(tools.widget);
}

ImagePanelConfig ImagePanel::config() const { return config_; }

void ImagePanel::setFrameFromDisplay(const QImage& image) {
  if (image.isNull()) {
    return;
  }
  base_image_ = image;
  updateRenderedFrame();
}

void ImagePanel::setConfig(const ImagePanelConfig& config) {
  config_ = config;
  main_queue_.clear();
  calibration_queue_.clear();
  resubscribeAll();
  applyConfigToUi();
}

void ImagePanel::cloneConfigFrom(const ImagePanelConfig& config) {
  config_ = config;
  base_image_ = QImage();
  base_timestamp_ns_ = 0;
  have_camera_info_ = false;
  have_fixed_to_optical_ = false;
  overlay_runtime_.clear();
  annotation_runtime_.clear();
  marker_runtime_.clear();
  video_decoder_.reset();
  resubscribeAll();
  applyConfigToUi();
}

void ImagePanel::applySettings(const ImagePanelConfig& config) {
  setConfig(config);
  emit configChanged();
}

void ImagePanel::applyConfigToUi() {
  if (view_ == nullptr) {
    return;
  }
  view_->setBackgroundColor(config_.background_color);
  view_->setLabelScale(config_.label_scale);
  updateRenderedFrame();
  syncSettingsToolState();
}

void ImagePanel::setSettingsVisible(bool visible) {
  config_.settings_visible = visible;
  syncSettingsToolState();
}

bool ImagePanel::settingsVisible() const { return config_.settings_visible; }

void ImagePanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ == nullptr) {
    return;
  }
  settings_button_->blockSignals(true);
  settings_button_->setChecked(checked);
  settings_button_->blockSignals(false);
}

void ImagePanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ == nullptr) {
    return;
  }
  expand_button_->blockSignals(true);
  expand_button_->setChecked(checked);
  expand_button_->blockSignals(false);
}

void ImagePanel::refreshSettingsChannels() {
  if (settings_widget_ != nullptr) {
    settings_widget_->refreshChannelLists();
  }
}

QWidget* ImagePanel::settingsWidgetForInspector() {
  return SettingsScrollForInspector(settings_scroll_);
}

void ImagePanel::recallSettingsWidget() {
  RecallSettingsScrollToContainer(settings_scroll_, settings_container_);
}

void ImagePanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ == nullptr) {
    return;
  }
  settings_widget_->blockSignals(true);
  settings_widget_->setConfig(config_);
  settings_widget_->blockSignals(false);
}

void ImagePanel::syncSettingsToolState() {
  setSettingsButtonChecked(config_.settings_visible);
}

void ImagePanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

void ImagePanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void ImagePanel::dragEnterEvent(QDragEnterEvent* event) {
  plot::PlotSeriesDragPayload payload;
  if (plot::ReadPlotSeriesDragPayload(event->mimeData(), &payload) &&
      !payload.channel.isEmpty() && payload.field_path.isEmpty()) {
    const std::string message_type = messageTypeForChannel(payload.channel.toStdString());
    if (display::isImageMessageType(message_type)) {
      event->acceptProposedAction();
      return;
    }
  }
  QWidget::dragEnterEvent(event);
}

void ImagePanel::dropEvent(QDropEvent* event) {
  plot::PlotSeriesDragPayload payload;
  if (plot::ReadPlotSeriesDragPayload(event->mimeData(), &payload) &&
      !payload.channel.isEmpty() && payload.field_path.isEmpty()) {
    const std::string message_type = messageTypeForChannel(payload.channel.toStdString());
    if (display::isImageMessageType(message_type)) {
      ImagePanelConfig updated = config_;
      updated.image_channel = payload.channel;
      setConfig(updated);
      emit configChanged();
      event->acceptProposedAction();
      return;
    }
  }
  QWidget::dropEvent(event);
}

std::string ImagePanel::messageTypeForChannel(const std::string& channel) const {
  if (manager_ == nullptr) {
    return {};
  }
  for (const integration::ChannelInfo& info : manager_->channels()) {
    if (info.channel_name == channel) {
      return info.message_type;
    }
  }
  return {};
}

QImage ImagePanel::decodePayload(const std::string& message_type,
                                 const std::string& payload) {
  const std::string decoded = integration::DecodeChannelPayload(payload);
  const auto try_parse_image = [&](const std::string& bytes) -> QImage {
    if (bytes.empty()) {
      return {};
    }
    automsgs::msgs::sensor_msgs::Image message;
    if (!message.ParseFromString(bytes) || message.width() == 0 ||
        message.height() == 0 || message.data().empty()) {
      return {};
    }
    return display::imageFromProto(message);
  };
  const auto try_parse_compressed = [&](const std::string& bytes) -> QImage {
    automsgs::msgs::sensor_msgs::CompressedImage message;
    if (!message.ParseFromString(bytes) && !message.ParseFromString(payload)) {
      return {};
    }
    const QString format =
        QString::fromStdString(message.format()).trimmed().toLower();
    if (isVideoFormat(format)) {
      return video_decoder_.decodePacket(
          format, reinterpret_cast<const std::byte*>(message.data().data()),
          message.data().size());
    }
    return display::compressedImageFromProto(message);
  };

  if (isVideoMessageType(message_type)) {
    if (!decoded.empty() && decoded.front() == '{') {
      return DecodeCompressedVideoJson(decoded, &video_decoder_);
    }
    return video_decoder_.decodePacket(
        QStringLiteral("h264"),
        reinterpret_cast<const std::byte*>(decoded.data()), decoded.size());
  }
  if (message_type.empty() ||
      commsgs::MessageTypesCompatible(
          message_type, "automsgs.msgs.sensor_msgs.Image") ||
      message_type == "sensor_msgs/Image") {
    for (const std::string& bytes : {decoded, payload}) {
      if (QImage image = try_parse_image(bytes); !image.isNull()) {
        return image;
      }
    }
    if (!message_type.empty()) {
      return {};
    }
  }
  if (message_type.empty() ||
      commsgs::MessageTypesCompatible(
          message_type, "automsgs.msgs.sensor_msgs.CompressedImage") ||
      message_type == "sensor_msgs/CompressedImage") {
    return try_parse_compressed(decoded);
  }
  return {};
}

void ImagePanel::handleMainPayload(const std::string& payload) {
  const std::string channel = config_.image_channel.toStdString();
  const std::string message_type = messageTypeForChannel(channel);
  const std::string decoded = integration::DecodeChannelPayload(payload);
  QImage image_q = decodePayload(message_type, payload);
  if (image_q.isNull()) {
    return;
  }
  if (message_type.empty() ||
      commsgs::MessageTypesCompatible(
          message_type, "automsgs.msgs.sensor_msgs.Image") ||
      message_type == "sensor_msgs/Image") {
    automsgs::msgs::sensor_msgs::Image message;
    if (message.ParseFromString(decoded) || message.ParseFromString(payload)) {
      base_timestamp_ns_ = HeaderTimestampNs(message.header());
    }
  } else if (commsgs::MessageTypesCompatible(
                 message_type, "automsgs.msgs.sensor_msgs.CompressedImage") ||
             message_type == "sensor_msgs/CompressedImage") {
    automsgs::msgs::sensor_msgs::CompressedImage message;
    if (message.ParseFromString(decoded) || message.ParseFromString(payload)) {
      base_timestamp_ns_ = HeaderTimestampNs(message.header());
    }
  } else if (isVideoMessageType(message_type) && !payload.empty() &&
             payload.front() == '{') {
    const QJsonDocument document =
        QJsonDocument::fromJson(QByteArray::fromStdString(payload));
    if (document.isObject()) {
      base_timestamp_ns_ =
          static_cast<qint64>(document.object()
                                  .value(QStringLiteral("timestamp"))
                                  .toObject()
                                  .value(QStringLiteral("sec"))
                                  .toVariant()
                                  .toLongLong()) *
              1000000000LL +
          document.object()
              .value(QStringLiteral("timestamp"))
              .toObject()
              .value(QStringLiteral("nsec"))
              .toVariant()
              .toLongLong();
    }
  }
  base_image_ = image_q;
  updateRenderedFrame();
}

void ImagePanel::handleCalibrationPayload(const std::string& payload) {
  automsgs::msgs::sensor_msgs::CameraInfo message;
  if (!message.ParseFromString(payload)) {
    return;
  }
  camera_info_ = message;
  camera_intrinsics_ = intrinsicsFromCameraInfo(message);
  have_camera_info_ = camera_intrinsics_.valid;
  updateFixedToOptical();
  updateRenderedFrame();
}

void ImagePanel::updateFixedToOptical() {
  have_fixed_to_optical_ = false;
  if (!have_camera_info_ || manager_ == nullptr) {
    return;
  }
  autoviz::transform::Buffer* tf_buffer = manager_->tfBuffer();
  if (tf_buffer == nullptr) {
    return;
  }
  const std::string camera_frame = camera_info_.header().frame_id();
  if (camera_frame.empty()) {
    return;
  }
  try {
    const auto zero_time = autoviz::commsgs::ZeroTime();
    const auto transform = tf_buffer->lookupTransform(
        manager_->fixedFrame(), camera_frame, zero_time);
    const QMatrix4x4 camera_to_fixed = TransformToMatrix(transform.transform());
    fixed_to_optical_ = fixedToOpticalMatrix(camera_info_, camera_to_fixed);
    have_fixed_to_optical_ = true;
  } catch (...) {
    have_fixed_to_optical_ = false;
  }
}

void ImagePanel::mergeAnnotationLayer(ImageAnnotationLayer* destination,
                                      const ImageAnnotationLayer& source) const {
  if (destination == nullptr) {
    return;
  }
  destination->polylines += source.polylines;
  destination->points += source.points;
  destination->texts += source.texts;
  if (destination->timestamp_ns == 0) {
    destination->timestamp_ns = source.timestamp_ns;
  }
}

void ImagePanel::handleMarkerPayload(int index, const std::string& payload) {
  if (index < 0 || index >= static_cast<int>(marker_runtime_.size()) ||
      !have_camera_info_ || !have_fixed_to_optical_ || manager_ == nullptr) {
    return;
  }
  MarkerRuntime& runtime = marker_runtime_[index];
  const std::string message_type = messageTypeForChannel(runtime.channel.toStdString());
  ImageAnnotationLayer merged;
  autoviz::transform::Buffer* tf_buffer = manager_->tfBuffer();
  const std::string fixed_frame = manager_->fixedFrame();

  auto projectMarker = [&](const automsgs::msgs::visualization_msgs::Marker& marker) {
    if (marker.action() == 2) {
      return;
    }
    mergeAnnotationLayer(
        &merged,
        projectMarkerToLayer(marker, camera_intrinsics_, fixed_to_optical_,
                             fixed_frame, tf_buffer));
  };

  if (message_type == "automsgs.msgs.visualization_msgs.Marker" ||
      message_type == "visualization_msgs/Marker") {
    automsgs::msgs::visualization_msgs::Marker marker;
    if (!marker.ParseFromString(payload)) {
      return;
    }
    runtime.timestamp_ns = HeaderTimestampNs(marker.header());
    projectMarker(marker);
  } else if (message_type == "automsgs.msgs.visualization_msgs.MarkerArray" ||
             message_type == "visualization_msgs/MarkerArray") {
    automsgs::msgs::visualization_msgs::MarkerArray array;
    if (!array.ParseFromString(payload)) {
      return;
    }
    if (array.markers_size() > 0) {
      runtime.timestamp_ns = HeaderTimestampNs(array.markers(0).header());
    }
    for (int i = 0; i < array.markers_size(); ++i) {
      projectMarker(array.markers(i));
    }
  } else {
    return;
  }

  runtime.layer = std::move(merged);
  updateRenderedFrame();
}

void ImagePanel::handleOverlayPayload(int index, const std::string& payload) {
  if (index < 0 || index >= static_cast<int>(overlay_runtime_.size())) {
    return;
  }
  OverlayRuntime& runtime = overlay_runtime_[index];
  const std::string message_type = messageTypeForChannel(runtime.config.channel.toStdString());
  runtime.image = decodePayload(message_type, payload);
  if (message_type == "automsgs.msgs.sensor_msgs.Image" ||
      message_type == "sensor_msgs/Image") {
    automsgs::msgs::sensor_msgs::Image message;
    if (message.ParseFromString(payload)) {
      runtime.timestamp_ns = HeaderTimestampNs(message.header());
    }
  }
  updateRenderedFrame();
}

void ImagePanel::handleAnnotationPayload(int index, const std::string& payload) {
  if (index < 0 || index >= static_cast<int>(annotation_runtime_.size())) {
    return;
  }
  AnnotationRuntime& runtime = annotation_runtime_[index];
  const std::string message_type = messageTypeForChannel(runtime.channel.toStdString());
  runtime.layer = ImageAnnotationParser::fromPayload(message_type, payload);
  runtime.timestamp_ns = runtime.layer.timestamp_ns;
  updateRenderedFrame();
}

void ImagePanel::updateRenderedFrame() {
  if (view_ == nullptr) {
    return;
  }
  QImage frame = base_image_;
  if (frame.isNull()) {
    view_->setStatusText(config_.image_channel.isEmpty()
                             ? tr("Select an image topic in Settings")
                             : config_.image_channel);
    view_->setFrame({});
    return;
  }

  frame = applyColorMode(frame, config_.color_mode, config_.color_min,
                         config_.color_max);
  if (config_.enable_undistort && have_camera_info_) {
    frame = undistortImage(frame, camera_intrinsics_);
  }
  frame = applyDisplayTransform(frame, config_.flip_horizontal,
                                config_.flip_vertical, config_.rotation);

  for (const OverlayRuntime& runtime : overlay_runtime_) {
    if (!runtime.config.enabled || runtime.image.isNull()) {
      continue;
    }
    if (config_.strict_time_sync && runtime.timestamp_ns != 0 &&
        base_timestamp_ns_ != 0 && runtime.timestamp_ns != base_timestamp_ns_) {
      continue;
    }
    QImage overlay = applyDisplayTransform(
        runtime.image, config_.flip_horizontal, config_.flip_vertical,
        config_.rotation);
    frame = compositeOverlay(frame, overlay, runtime.config);
  }

  view_->setStatusText(config_.image_channel);
  view_->setFrame(frame);

  QVector<ImageAnnotationLayer> layers;
  layers.reserve(static_cast<int>(annotation_runtime_.size() +
                                  marker_runtime_.size()));
  for (const AnnotationRuntime& runtime : annotation_runtime_) {
    if (config_.strict_time_sync && runtime.timestamp_ns != 0 &&
        base_timestamp_ns_ != 0 && runtime.timestamp_ns != base_timestamp_ns_) {
      continue;
    }
    layers.push_back(runtime.layer);
  }
  for (const MarkerRuntime& runtime : marker_runtime_) {
    if (config_.strict_time_sync && runtime.timestamp_ns != 0 &&
        base_timestamp_ns_ != 0 && runtime.timestamp_ns != base_timestamp_ns_) {
      continue;
    }
    layers.push_back(runtime.layer);
  }
  view_->setAnnotationLayers(layers);
}

void ImagePanel::publishPixel(const QString& channel, int x, int y) const {
  if (channel.isEmpty() || manager_ == nullptr) {
    return;
  }
  auto node = manager_->autolinkNode();
  if (node == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::Point point;
  point.set_x(static_cast<double>(x));
  point.set_y(static_cast<double>(y));
  point.set_z(0.0);
  std::string payload;
  if (!point.SerializeToString(&payload)) {
    return;
  }
  static thread_local std::unordered_map<std::string,
      std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>>>
      writers;
  const std::string channel_std = channel.toStdString();
  auto& writer = writers[channel_std];
  if (writer == nullptr) {
    writer = node->CreateWriter<::autolink::message::RawMessage>(channel_std);
  }
  if (writer == nullptr) {
    return;
  }
  auto message = std::make_shared<::autolink::message::RawMessage>();
  message->message = payload;
  writer->Write(message);
}

void ImagePanel::unsubscribeMain() {
  if (main_subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(main_subscription_id_);
    main_subscription_id_ = 0;
  }
  main_queue_.clear();
}

void ImagePanel::subscribeMain() {
  unsubscribeMain();
  const std::string channel = config_.image_channel.toStdString();
  if (channel.empty()) {
    return;
  }
  // Callbacks run on the autolink scheduler thread — only enqueue here.
  main_subscription_id_ = integration::ChannelReaderRegistry::instance().subscribe(
      channel, [this](const std::string& payload) { main_queue_.push(payload); });
}

void ImagePanel::unsubscribeOverlays() {
  for (OverlayRuntime& runtime : overlay_runtime_) {
    if (runtime.subscription_id != 0) {
      integration::ChannelReaderRegistry::instance().unsubscribe(
          runtime.subscription_id);
      runtime.subscription_id = 0;
    }
  }
  overlay_runtime_.clear();
}

void ImagePanel::subscribeOverlays() {
  unsubscribeOverlays();
  overlay_runtime_.reserve(static_cast<std::size_t>(config_.overlays.size()));
  for (int i = 0; i < config_.overlays.size(); ++i) {
    OverlayRuntime runtime;
    runtime.config = config_.overlays.at(i);
    overlay_runtime_.push_back(std::move(runtime));
    const std::string channel =
        overlay_runtime_.back().config.channel.toStdString();
    if (channel.empty() || !overlay_runtime_.back().config.enabled) {
      continue;
    }
    overlay_runtime_.back().subscription_id =
        integration::ChannelReaderRegistry::instance().subscribe(
            channel, [this, i](const std::string& payload) {
              if (i >= 0 && static_cast<std::size_t>(i) < overlay_runtime_.size()) {
                overlay_runtime_[static_cast<std::size_t>(i)].queue.push(payload);
              }
            });
  }
}

void ImagePanel::unsubscribeAnnotations() {
  for (AnnotationRuntime& runtime : annotation_runtime_) {
    if (runtime.subscription_id != 0) {
      integration::ChannelReaderRegistry::instance().unsubscribe(
          runtime.subscription_id);
      runtime.subscription_id = 0;
    }
  }
  annotation_runtime_.clear();
}

void ImagePanel::subscribeAnnotations() {
  unsubscribeAnnotations();
  annotation_runtime_.reserve(
      static_cast<std::size_t>(config_.annotation_channels.size()));
  for (int i = 0; i < config_.annotation_channels.size(); ++i) {
    AnnotationRuntime runtime;
    runtime.channel = config_.annotation_channels.at(i);
    annotation_runtime_.push_back(std::move(runtime));
    const std::string channel =
        annotation_runtime_.back().channel.toStdString();
    if (channel.empty()) {
      continue;
    }
    annotation_runtime_.back().subscription_id =
        integration::ChannelReaderRegistry::instance().subscribe(
            channel, [this, i](const std::string& payload) {
              if (i >= 0 &&
                  static_cast<std::size_t>(i) < annotation_runtime_.size()) {
                annotation_runtime_[static_cast<std::size_t>(i)].queue.push(
                    payload);
              }
            });
  }
}

void ImagePanel::resubscribeAll() {
  subscribeMain();
  subscribeOverlays();
  subscribeAnnotations();
  subscribeCalibration();
  subscribeMarkers();
  syncSettingsWidgetFromConfig();
}

void ImagePanel::unsubscribeCalibration() {
  if (calibration_subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(
        calibration_subscription_id_);
    calibration_subscription_id_ = 0;
  }
  calibration_queue_.clear();
}

void ImagePanel::subscribeCalibration() {
  unsubscribeCalibration();
  const std::string channel = config_.calibration_channel.toStdString();
  if (channel.empty()) {
    have_camera_info_ = false;
    have_fixed_to_optical_ = false;
    return;
  }
  calibration_subscription_id_ =
      integration::ChannelReaderRegistry::instance().subscribe(
          channel, [this](const std::string& payload) {
            calibration_queue_.push(payload);
          });
}

void ImagePanel::unsubscribeMarkers() {
  for (MarkerRuntime& runtime : marker_runtime_) {
    if (runtime.subscription_id != 0) {
      integration::ChannelReaderRegistry::instance().unsubscribe(
          runtime.subscription_id);
      runtime.subscription_id = 0;
    }
  }
  marker_runtime_.clear();
}

void ImagePanel::subscribeMarkers() {
  unsubscribeMarkers();
  marker_runtime_.reserve(
      static_cast<std::size_t>(config_.marker_channels.size()));
  for (int i = 0; i < config_.marker_channels.size(); ++i) {
    MarkerRuntime runtime;
    runtime.channel = config_.marker_channels.at(i);
    marker_runtime_.push_back(std::move(runtime));
    const std::string channel = marker_runtime_.back().channel.toStdString();
    if (channel.empty()) {
      continue;
    }
    marker_runtime_.back().subscription_id =
        integration::ChannelReaderRegistry::instance().subscribe(
            channel, [this, i](const std::string& payload) {
              if (i >= 0 &&
                  static_cast<std::size_t>(i) < marker_runtime_.size()) {
                marker_runtime_[static_cast<std::size_t>(i)].queue.push(payload);
              }
            });
  }
}

void ImagePanel::drainIncomingQueues() {
  if (auto payload = main_queue_.takeLatest()) {
    handleMainPayload(*payload);
  }
  if (auto payload = calibration_queue_.takeLatest()) {
    handleCalibrationPayload(*payload);
  }
  for (std::size_t i = 0; i < overlay_runtime_.size(); ++i) {
    if (auto payload = overlay_runtime_[i].queue.takeLatest()) {
      handleOverlayPayload(static_cast<int>(i), *payload);
    }
  }
  for (std::size_t i = 0; i < annotation_runtime_.size(); ++i) {
    if (auto payload = annotation_runtime_[i].queue.takeLatest()) {
      handleAnnotationPayload(static_cast<int>(i), *payload);
    }
  }
  for (std::size_t i = 0; i < marker_runtime_.size(); ++i) {
    if (auto payload = marker_runtime_[i].queue.takeLatest()) {
      handleMarkerPayload(static_cast<int>(i), *payload);
    }
  }
}

void ImagePanel::tick() { onFrameTick(); }

void ImagePanel::onFrameTick() {
  if (main_subscription_id_ == 0 && !config_.image_channel.isEmpty()) {
    subscribeMain();
  }
  // Handlers already call updateRenderedFrame() when a payload arrives.
  drainIncomingQueues();
}

}  // namespace image
}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_panel.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <QAbstractSpinBox>
#include <QButtonGroup>
#include <QDateTime>
#include <QDragEnterEvent>
#include <QDoubleSpinBox>
#include <QDropEvent>
#include <QFileDialog>
#include <QFocusEvent>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QLabel>
#include <QMessageBox>
#include <QMimeData>
#include <QPainter>
#include <QPainterPath>
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
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/image_utils.hpp"
#include "autoviz/integration/channel_payload.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/message_queue.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/image/image_analysis.hpp"
#include "autoviz/ui/image/image_annotation_parser.hpp"
#include "autoviz/ui/image/image_calibration_utils.hpp"
#include "autoviz/ui/image/image_histogram_widget.hpp"
#include "autoviz/ui/image/image_marker_projection.hpp"
#include "autoviz/ui/image/image_point_cloud_projection.hpp"
#include "autoviz/ui/image/image_processing.hpp"
#include "autoviz/ui/image/image_profile_widget.hpp"
#include "autoviz/ui/image/image_settings_widget.hpp"
#include "autoviz/ui/image/image_video_decoder.hpp"
#include "autoviz/ui/image/image_view_widget.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"
#include "autoviz/ui/panel/title_tools.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"

namespace autoviz {
namespace image {
namespace {

qint64 HeaderTimestampNs(const automsgs::msgs::std_msgs::Header& header) {
  return static_cast<qint64>(header.stamp().sec()) * 1000000000LL +
         static_cast<qint64>(header.stamp().nanosec());
}

/** Translucent cyan-white glass card floating over the image. */
class ImageFloatingChrome : public QFrame {
 public:
  explicit ImageFloatingChrome(QWidget* parent = nullptr) : QFrame(parent) {
    setObjectName(QStringLiteral("ImageFloatingChrome"));
    setAttribute(Qt::WA_TranslucentBackground, true);
    setAutoFillBackground(false);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
  }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const glass::ShellTokens t = glass::Shell();
    const QRectF card = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    QPainterPath path;
    path.addRoundedRect(card, 14.0, 14.0);

    // See-through milky glass (image shows through).
    painter.fillPath(path, QColor(255, 255, 255, 72));
    QLinearGradient body(card.topLeft(), card.bottomLeft());
    body.setColorAt(0.0, QColor(240, 253, 250, 90));
    body.setColorAt(0.55, QColor(255, 255, 255, 48));
    body.setColorAt(1.0, QColor(204, 251, 241, 36));
    painter.fillPath(path, body);

    QColor sheen_end = t.glass_sheen;
    sheen_end.setAlpha(0);
    QLinearGradient sheen(card.topLeft(),
                          QPointF(card.left(), card.top() + 26.0));
    sheen.setColorAt(0.0, QColor(255, 255, 255, 110));
    sheen.setColorAt(1.0, sheen_end);
    painter.fillPath(path, sheen);

    painter.setPen(QPen(QColor(8, 145, 178, 70), 1.0));
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);
  }
};

/** iOS-style translucent glass pill for restoring the analysis bar. */
class ImageFloatingRevealButton : public QToolButton {
 public:
  explicit ImageFloatingRevealButton(QWidget* parent = nullptr)
      : QToolButton(parent) {
    setObjectName(QStringLiteral("ImageFloatingReveal"));
    setAttribute(Qt::WA_TranslucentBackground, true);
    setAutoFillBackground(false);
    setCursor(Qt::PointingHandCursor);
  }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const QRectF card = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    QPainterPath path;
    path.addRoundedRect(card, 14.0, 14.0);

    const bool hover = underMouse();
    // iOS material: light frosted glass over content.
    painter.fillPath(path, QColor(255, 255, 255, hover ? 92 : 58));
    painter.fillPath(path, QColor(199, 244, 246, hover ? 55 : 32));
    QLinearGradient sheen(card.topLeft(),
                          QPointF(card.left(), card.top() + card.height() * 0.55));
    sheen.setColorAt(0.0, QColor(255, 255, 255, hover ? 130 : 95));
    sheen.setColorAt(1.0, QColor(255, 255, 255, 0));
    painter.fillPath(path, sheen);

    painter.setPen(QPen(QColor(255, 255, 255, hover ? 170 : 130), 1.0));
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);

    QFont font = this->font();
    font.setPixelSize(11);
    font.setWeight(QFont::DemiBold);
    painter.setFont(font);
    painter.setPen(hover ? QColor(8, 145, 178, 230) : QColor(30, 41, 59, 210));
    painter.drawText(rect(), Qt::AlignCenter, text());
  }
};

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
  ApplyPanelShell(this);

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

  image_host_ = new QWidget(this);
  auto* host_layout = new QGridLayout(image_host_);
  host_layout->setContentsMargins(0, 0, 0, 0);
  host_layout->setSpacing(0);

  view_ = new ImageViewWidget(image_host_);
  host_layout->addWidget(view_, 0, 0);

  overlay_wrap_ = new QWidget(image_host_);
  overlay_wrap_->setAttribute(Qt::WA_TranslucentBackground, true);
  overlay_wrap_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
  auto* wrap_layout = new QVBoxLayout(overlay_wrap_);
  wrap_layout->setContentsMargins(10, 0, 10, 8);
  wrap_layout->setSpacing(6);

  overlay_chrome_ = new ImageFloatingChrome(overlay_wrap_);
  overlay_chrome_->setStyleSheet(
      style::sheet(QStringLiteral("image/floating_chrome")));
  auto* chrome_layout = new QVBoxLayout(overlay_chrome_);
  chrome_layout->setContentsMargins(10, 8, 10, 8);
  chrome_layout->setSpacing(6);

  tool_bar_ = new QWidget(overlay_chrome_);
  tool_bar_->setAttribute(Qt::WA_TranslucentBackground, true);
  auto* tool_layout = new QHBoxLayout(tool_bar_);
  tool_layout->setContentsMargins(0, 0, 0, 0);
  tool_layout->setSpacing(3);
  tool_group_ = new QButtonGroup(this);
  tool_group_->setExclusive(true);

  auto make_tool = [&](const QString& text, const QString& tip,
                       ImageViewTool tool) -> QToolButton* {
    auto* button = new QToolButton(tool_bar_);
    button->setText(text);
    button->setToolTip(tip);
    button->setCheckable(true);
    button->setAutoRaise(true);
    button->setToolButtonStyle(Qt::ToolButtonTextOnly);
    tool_group_->addButton(button, static_cast<int>(tool));
    tool_layout->addWidget(button);
    return button;
  };
  make_tool(tr("Probe"), tr("Pixel probe (default)"), ImageViewTool::kProbe)
      ->setChecked(true);
  make_tool(tr("Measure"), tr("Click two points to measure distance"),
            ImageViewTool::kMeasure);
  make_tool(tr("ROI"), tr("Drag a rectangle for region statistics"),
            ImageViewTool::kRoi);
  make_tool(tr("Profile"), tr("Click two points for a luminance profile"),
            ImageViewTool::kProfile);

  auto* hist_toggle = new QToolButton(tool_bar_);
  hist_toggle->setText(tr("Histogram"));
  hist_toggle->setToolTip(tr("Show / hide luminance histogram"));
  hist_toggle->setCheckable(true);
  hist_toggle->setChecked(false);
  hist_toggle->setAutoRaise(true);
  tool_layout->addWidget(hist_toggle);

  measure_depth_label_ = new QLabel(tr("Depth"), tool_bar_);
  tool_layout->addWidget(measure_depth_label_);
  measure_depth_spin_ = new QDoubleSpinBox(tool_bar_);
  measure_depth_spin_->setRange(0.05, 500.0);
  measure_depth_spin_->setDecimals(2);
  measure_depth_spin_->setSuffix(tr(" m"));
  measure_depth_spin_->setValue(1.0);
  measure_depth_spin_->setButtonSymbols(QAbstractSpinBox::NoButtons);
  measure_depth_spin_->setAlignment(Qt::AlignCenter);
  measure_depth_spin_->setToolTip(
      tr("Assumed optical-plane depth for metric Measure"));
  measure_depth_spin_->setMaximumWidth(84);
  tool_layout->addWidget(measure_depth_spin_);
  measure_depth_label_->hide();
  measure_depth_spin_->hide();
  tool_layout->addStretch(1);

  analysis_label_ = new QLabel(tool_bar_);
  analysis_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  analysis_label_->setMinimumWidth(80);
  tool_layout->addWidget(analysis_label_, 1);

  overlay_hide_button_ = new QToolButton(tool_bar_);
  overlay_hide_button_->setText(tr("Hide"));
  overlay_hide_button_->setToolTip(tr("Hide analysis bar"));
  overlay_hide_button_->setAutoRaise(true);
  tool_layout->addWidget(overlay_hide_button_);
  chrome_layout->addWidget(tool_bar_);

  analysis_strip_ = new QWidget(overlay_chrome_);
  analysis_strip_->setAttribute(Qt::WA_TranslucentBackground, true);
  analysis_strip_->hide();
  auto* analysis_layout = new QVBoxLayout(analysis_strip_);
  analysis_layout->setContentsMargins(0, 0, 0, 0);
  analysis_layout->setSpacing(4);
  histogram_widget_ = new ImageHistogramWidget(analysis_strip_);
  histogram_widget_->setMaximumHeight(48);
  histogram_widget_->hide();
  analysis_layout->addWidget(histogram_widget_);
  profile_widget_ = new ImageProfileWidget(analysis_strip_);
  profile_widget_->setMaximumHeight(48);
  profile_widget_->hide();
  analysis_layout->addWidget(profile_widget_);
  chrome_layout->addWidget(analysis_strip_);
  wrap_layout->addWidget(overlay_chrome_);

  overlay_reveal_button_ = new ImageFloatingRevealButton(overlay_wrap_);
  overlay_reveal_button_->setText(tr("Tools"));
  overlay_reveal_button_->setToolTip(tr("Show analysis tools"));
  overlay_reveal_button_->setAutoRaise(true);
  overlay_reveal_button_->setStyleSheet(
      style::sheet(QStringLiteral("image/floating_reveal")));
  overlay_reveal_button_->hide();
  wrap_layout->addWidget(overlay_reveal_button_, 0, Qt::AlignHCenter);

  host_layout->addWidget(overlay_wrap_, 0, 0, Qt::AlignBottom);
  overlay_wrap_->raise();
  root->addWidget(image_host_, 1);
  setFloatingChromeVisible(false);

  connect(tool_group_, &QButtonGroup::idClicked, this, [this](int id) {
    setActiveTool(static_cast<ImageViewTool>(id));
  });
  connect(hist_toggle, &QToolButton::toggled, this, [this](bool on) {
    histogram_visible_ = on;
    updateAnalysisStripVisibility();
    if (on) {
      refreshAnalysisPanel();
    }
  });
  connect(overlay_hide_button_, &QToolButton::clicked, this,
          [this]() { setFloatingChromeVisible(false); });
  connect(overlay_reveal_button_, &QToolButton::clicked, this,
          [this]() { setFloatingChromeVisible(true); });
  connect(measure_depth_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this](double depth) {
            if (view_ != nullptr) {
              view_->setMeasureCalibration(camera_intrinsics_, depth);
            }
          });
  connect(histogram_widget_, &ImageHistogramWidget::rangeChanged, this,
          [this](double min_v, double max_v) {
            if (std::abs(config_.color_min - min_v) < 1e-6 &&
                std::abs(config_.color_max - max_v) < 1e-6) {
              return;
            }
            config_.color_min = min_v;
            config_.color_max = max_v;
            if (config_.color_mode == ImageColorMode::kOff) {
              config_.color_mode = ImageColorMode::kTurbo;
            }
            syncSettingsWidgetFromConfig();
            updateRenderedFrame();
            emit configChanged();
          });

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
  connect(settings_widget_, &ImageSettingsWidget::moveOverlayRequested, this,
          [this](int index, int delta) {
            ImagePanelConfig updated = config_;
            const int target = index + delta;
            if (index < 0 || index >= updated.overlays.size() || target < 0 ||
                target >= updated.overlays.size()) {
              return;
            }
            updated.overlays.swapItemsAt(index, target);
            setConfig(updated);
            emit configChanged();
          });

  connect(view_, &ImageViewWidget::pixelClicked, this,
          [this](int x, int y) { publishPixel(config_.click_publish_channel, x, y); });
  connect(view_, &ImageViewWidget::pixelHovered, this,
          [this](int x, int y) { publishPixel(config_.hover_publish_channel, x, y); });
  connect(view_, &ImageViewWidget::exportPngRequested, this,
          &ImagePanel::exportImageAsPng);
  connect(view_, &ImageViewWidget::measureCompleted, this,
          [this](QPoint, QPoint, double pixels, double meters) {
            const auto opt = meters >= 0.0 ? std::optional<double>(meters)
                                           : std::nullopt;
            const QString text = formatMeasureReadout(pixels, opt);
            if (analysis_label_ != nullptr) {
              analysis_label_->setText(text);
            }
            if (view_ != nullptr) {
              view_->setAnalysisText(text);
            }
          });
  connect(view_, &ImageViewWidget::profileCompleted, this,
          [this](QPoint a, QPoint b) {
            if (view_ == nullptr || profile_widget_ == nullptr) {
              return;
            }
            updateAnalysisStripVisibility();
            const QVector<double> samples =
                sampleLineProfile(view_->frame(), a, b);
            profile_widget_->setSamples(samples);
            const QString text =
                QStringLiteral("Profile: %1 samples  %2 px")
                    .arg(samples.size())
                    .arg(pixelDistance(a, b), 0, 'f', 1);
            if (analysis_label_ != nullptr) {
              analysis_label_->setText(text);
            }
            view_->setAnalysisText(text);
          });
  connect(view_, &ImageViewWidget::roiChanged, this, [this](QRect roi) {
    refreshAnalysisPanel();
    if (view_ == nullptr || view_->frame().isNull()) {
      return;
    }
    if (!roi.isValid() || roi.isEmpty()) {
      if (analysis_label_ != nullptr) {
        analysis_label_->clear();
      }
      return;
    }
    const ImageRoiStats stats = computeRoiStats(view_->frame(), roi);
    const QString text = formatRoiStats(stats);
    if (analysis_label_ != nullptr) {
      analysis_label_->setText(text);
    }
    if (view_ != nullptr) {
      view_->setAnalysisText(text);
    }
  });

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
  unsubscribePointClouds();
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

  callbacks.download_image_png = [this]() { exportImageAsPng(false); };

  const PanelTitleBarTools tools =
      CreatePanelTitleBarTools(dock, callbacks, options);
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
  noteFrameArrival();
  updateRenderedFrame();
}

void ImagePanel::exportImageAsPng(bool with_annotations) {
  if (view_ == nullptr) {
    return;
  }
  const QImage image = view_->renderExportImage(with_annotations);
  if (image.isNull()) {
    QMessageBox::information(this, tr("Export"),
                             tr("No image frame to export."));
    return;
  }
  const QString default_name =
      config_.title.isEmpty() ? tr("image.png")
                              : config_.title + QStringLiteral(".png");
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Download image as PNG"), default_name,
      tr("PNG images (*.png)"));
  if (path.isEmpty()) {
    return;
  }
  if (!image.save(path, "PNG")) {
    QMessageBox::warning(this, tr("Export failed"),
                         tr("Could not write to %1").arg(path));
  }
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
  noteFrameArrival();
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
  runtime.message_type = message_type;
  runtime.last_payload = payload;
  runtime.layer = ImageAnnotationParser::fromPayload(message_type, payload);
  runtime.timestamp_ns = runtime.layer.timestamp_ns;
  updateRenderedFrame();
}

void ImagePanel::refreshAnalysisPanel() {
  if (histogram_widget_ == nullptr || view_ == nullptr) {
    return;
  }
  const QImage frame = view_->frame();
  if (frame.isNull()) {
    histogram_widget_->setHistogram({});
    return;
  }
  if (!histogram_visible_) {
    return;
  }
  const QRect roi = view_->analysisRoi();
  histogram_widget_->setHistogram(computeLumaHistogram(frame, roi));
  histogram_widget_->setRange(config_.color_min, config_.color_max);
}

void ImagePanel::updateRenderedFrame() {
  if (view_ == nullptr) {
    return;
  }
  QImage frame = base_image_;
  if (frame.isNull()) {
    display_fps_ = 0.0;
    frame_arrival_ms_.clear();
    ImageViewHud hud;
    hud.title = config_.image_channel.isEmpty()
                    ? tr("Select an image topic in Settings")
                    : config_.image_channel;
    view_->setHud(hud);
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

  QStringList overlay_warnings;
  for (const OverlayRuntime& runtime : overlay_runtime_) {
    if (!runtime.config.enabled || runtime.image.isNull()) {
      continue;
    }
    if (config_.strict_time_sync && runtime.timestamp_ns != 0 &&
        base_timestamp_ns_ != 0 && runtime.timestamp_ns != base_timestamp_ns_) {
      continue;
    }
    if (!base_image_.isNull() &&
        (runtime.image.width() != base_image_.width() ||
         runtime.image.height() != base_image_.height())) {
      overlay_warnings << tr("%1 size %2×%3 ≠ %4×%5")
                              .arg(runtime.config.channel)
                              .arg(runtime.image.width())
                              .arg(runtime.image.height())
                              .arg(base_image_.width())
                              .arg(base_image_.height());
    }
    QImage overlay = applyDisplayTransform(
        runtime.image, config_.flip_horizontal, config_.flip_vertical,
        config_.rotation);
    frame = compositeOverlay(frame, overlay, runtime.config);
  }

  view_->setFrame(frame);
  if (view_ != nullptr) {
    const double depth =
        measure_depth_spin_ != nullptr ? measure_depth_spin_->value() : 1.0;
    view_->setMeasureCalibration(camera_intrinsics_, depth);
  }
  updateViewHud();
  refreshAnalysisPanel();
  if (analysis_label_ != nullptr && !overlay_warnings.isEmpty()) {
    analysis_label_->setText(tr("Overlay mismatch: %1")
                                 .arg(overlay_warnings.join(QStringLiteral("; "))));
  }

  QVector<ImageAnnotationLayer> layers;
  layers.reserve(static_cast<int>(annotation_runtime_.size() +
                                  marker_runtime_.size() +
                                  point_cloud_runtime_.size()));
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
  for (const PointCloudRuntime& runtime : point_cloud_runtime_) {
    if (config_.strict_time_sync && runtime.timestamp_ns != 0 &&
        base_timestamp_ns_ != 0 && runtime.timestamp_ns != base_timestamp_ns_) {
      continue;
    }
    layers.push_back(runtime.layer);
  }
  view_->setAnnotationLayers(layers);
}

void ImagePanel::noteFrameArrival() {
  const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
  frame_arrival_ms_.push_back(now_ms);
  constexpr qint64 kWindowMs = 2000;
  while (!frame_arrival_ms_.isEmpty() &&
         now_ms - frame_arrival_ms_.front() > kWindowMs) {
    frame_arrival_ms_.removeFirst();
  }
  if (frame_arrival_ms_.size() >= 2) {
    const qint64 span_ms =
        frame_arrival_ms_.back() - frame_arrival_ms_.front();
    if (span_ms > 0) {
      display_fps_ = (frame_arrival_ms_.size() - 1) * 1000.0 /
                     static_cast<double>(span_ms);
    }
  }
}

void ImagePanel::updateViewHud() {
  if (view_ == nullptr) {
    return;
  }
  ImageViewHud hud;
  hud.title = config_.image_channel.isEmpty() ? config_.title
                                              : config_.image_channel;

  QStringList meta_parts;
  if (!base_image_.isNull()) {
    meta_parts << QStringLiteral("%1×%2")
                      .arg(base_image_.width())
                      .arg(base_image_.height());
  }
  if (display_fps_ > 0.05) {
    meta_parts << tr("%1 fps").arg(display_fps_, 0, 'f', 1);
  }
  if (have_camera_info_) {
    meta_parts << tr("calib");
  }
  if (config_.enable_undistort && have_camera_info_) {
    meta_parts << tr("undistort");
  } else if (config_.enable_undistort && !have_camera_info_) {
    meta_parts << tr("undistort (no calib)");
  }
  if (config_.strict_time_sync) {
    meta_parts << tr("sync");
  }
  hud.meta = meta_parts.join(QStringLiteral(" · "));
  view_->setHud(hud);
}

void ImagePanel::setActiveTool(ImageViewTool tool) {
  if (view_ != nullptr) {
    view_->setTool(tool);
  }
  const bool measure = tool == ImageViewTool::kMeasure;
  if (measure_depth_label_ != nullptr) {
    measure_depth_label_->setVisible(measure);
  }
  if (measure_depth_spin_ != nullptr) {
    measure_depth_spin_->setVisible(measure);
  }
  if (analysis_label_ != nullptr && tool == ImageViewTool::kProbe) {
    analysis_label_->clear();
  }
  updateAnalysisStripVisibility();
}

void ImagePanel::updateAnalysisStripVisibility() {
  const bool show_profile =
      view_ != nullptr && view_->tool() == ImageViewTool::kProfile;
  if (histogram_widget_ != nullptr) {
    histogram_widget_->setVisible(histogram_visible_);
  }
  if (profile_widget_ != nullptr) {
    profile_widget_->setVisible(show_profile);
  }
  if (analysis_strip_ != nullptr) {
    analysis_strip_->setVisible(histogram_visible_ || show_profile);
  }
}

void ImagePanel::setFloatingChromeVisible(bool visible) {
  floating_chrome_visible_ = visible;
  if (overlay_chrome_ != nullptr) {
    overlay_chrome_->setVisible(visible);
  }
  if (overlay_reveal_button_ != nullptr) {
    overlay_reveal_button_->setVisible(!visible);
  }
  if (overlay_wrap_ != nullptr) {
    overlay_wrap_->raise();
    overlay_wrap_->updateGeometry();
  }
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
  subscribePointClouds();
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

void ImagePanel::unsubscribePointClouds() {
  for (PointCloudRuntime& runtime : point_cloud_runtime_) {
    if (runtime.subscription_id != 0) {
      integration::ChannelReaderRegistry::instance().unsubscribe(
          runtime.subscription_id);
      runtime.subscription_id = 0;
    }
  }
  point_cloud_runtime_.clear();
}

void ImagePanel::subscribePointClouds() {
  unsubscribePointClouds();
  point_cloud_runtime_.reserve(
      static_cast<std::size_t>(config_.point_cloud_channels.size()));
  for (int i = 0; i < config_.point_cloud_channels.size(); ++i) {
    PointCloudRuntime runtime;
    runtime.channel = config_.point_cloud_channels.at(i);
    point_cloud_runtime_.push_back(std::move(runtime));
    const std::string channel =
        point_cloud_runtime_.back().channel.toStdString();
    if (channel.empty()) {
      continue;
    }
    point_cloud_runtime_.back().subscription_id =
        integration::ChannelReaderRegistry::instance().subscribe(
            channel, [this, i](const std::string& payload) {
              if (i >= 0 &&
                  static_cast<std::size_t>(i) < point_cloud_runtime_.size()) {
                point_cloud_runtime_[static_cast<std::size_t>(i)].queue.push(
                    payload);
              }
            });
  }
}

void ImagePanel::handlePointCloudPayload(int index, const std::string& payload) {
  if (index < 0 || index >= static_cast<int>(point_cloud_runtime_.size()) ||
      !have_camera_info_ || !have_fixed_to_optical_ || manager_ == nullptr) {
    return;
  }
  PointCloudRuntime& runtime = point_cloud_runtime_[static_cast<std::size_t>(index)];
  automsgs::msgs::sensor_msgs::PointCloud2 cloud;
  if (!cloud.ParseFromString(payload)) {
    return;
  }
  runtime.layer = projectPointCloudToLayer(
      cloud, camera_intrinsics_, fixed_to_optical_, manager_->fixedFrame(),
      manager_->tfBuffer());
  runtime.timestamp_ns = runtime.layer.timestamp_ns;
  updateRenderedFrame();
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
  for (std::size_t i = 0; i < point_cloud_runtime_.size(); ++i) {
    if (auto payload = point_cloud_runtime_[i].queue.takeLatest()) {
      handlePointCloudPayload(static_cast<int>(i), *payload);
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

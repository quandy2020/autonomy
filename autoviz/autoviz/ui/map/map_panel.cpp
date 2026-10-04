/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_panel.hpp"

#include <QAction>
#include <QApplication>
#include <QClipboard>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QMimeData>
#include <QScrollArea>
#include <QSet>
#include <QToolButton>
#include <QUrl>
#include <QVBoxLayout>

#include <chrono>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/map/map_message_ingest.hpp"
#include "autoviz/ui/map/map_settings_widget.hpp"
#include "autoviz/ui/map/map_viewport_widget.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/panel/title_tools.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {
namespace map {
namespace {

}  // namespace

MapPanel::MapPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultMapPanelConfig()), QWidget(parent) {
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
  settings_scroll_->setObjectName(QString::fromLatin1(AppThemeIds::kSettingsScroll));
  settings_scroll_->setWidgetResizable(true);
  settings_scroll_->setFrameShape(QFrame::NoFrame);
  settings_scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  settings_widget_ = new MapSettingsWidget(manager_, settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
  settings_layout->addWidget(settings_scroll_);

  view_ = new MapViewportWidget(this);
  root->addWidget(view_, 1);
  QLabel* status = nullptr;
  auto* footer = MakePanelFooter(this, &status);
  status_label_ = status;
  root->addWidget(footer);
  connect(view_, &MapViewportWidget::geoJsonSourcesChanged, this, [this]() {
    if (view_ == nullptr) {
      return;
    }
    config_.geojson_sources = view_->geoJsonSources();
    syncSettingsWidgetFromConfig();
    updateStatusBar();
    emit configChanged();
  });
  connect(view_, &MapViewportWidget::mapStatusChanged, this,
          [this](const QString& text) {
            map_status_extra_ = text;
            updateStatusBar();
          });
  connect(view_, &MapViewportWidget::cursorGeoChanged, this,
          [this](double latitude, double longitude) {
            cursor_latitude_ = latitude;
            cursor_longitude_ = longitude;
            updateStatusBar();
          });
  connect(view_, &MapViewportWidget::hideTopicLayerRequested, this,
          [this](const QString& channel) {
            for (MapTopicLayerConfig& layer : config_.topic_layers) {
              if (layer.channel == channel) {
                layer.enabled = false;
              }
            }
            syncSettingsWidgetFromConfig();
            refreshViewport();
            updateStatusBar();
            emit configChanged();
          });

  follow_timer_.setInterval(100);
  connect(&follow_timer_, &QTimer::timeout, this, &MapPanel::onFollowTick);

  connect(settings_widget_, &MapSettingsWidget::configChanged, this, [this]() {
    config_ = settings_widget_->config();
    applyConfigToUi();
    resubscribeAll();
    emit configChanged();
  });
  connect(view_, &MapViewportWidget::viewChanged, this, &MapPanel::onViewChanged);
  connect(view_, &MapViewportWidget::selectionChanged, settings_widget_,
          &MapSettingsWidget::setSelection);
  connect(settings_widget_, &MapSettingsWidget::planVertexEdited, this,
          [this](int kind, int index, double latitude, double longitude) {
            if (view_ != nullptr) {
              view_->setPlanVertex(kind, index, latitude, longitude);
            }
          });
  connect(settings_widget_, &MapSettingsWidget::planVertexRemoved, this,
          [this](int kind, int index) {
            if (view_ != nullptr) {
              view_->removePlanVertex(kind, index);
            }
          });
  connect(view_, &MapViewportWidget::planChanged, this, [this]() {
    if (view_ == nullptr) {
      return;
    }
    const MapPanelConfig view_config = view_->config();
    config_.waypoints = view_config.waypoints;
    config_.geofence = view_config.geofence;
    config_.rally_points = view_config.rally_points;
    config_.edit_tool = view_config.edit_tool;
    syncSettingsWidgetFromConfig();
    updateStatusBar();
    emit configChanged();
  });
  connect(settings_widget_, &MapSettingsWidget::offlineDownloadRequested, this,
          [this](int min_zoom, int max_zoom) {
            if (view_ != nullptr) {
              view_->downloadVisibleTiles(min_zoom, max_zoom);
            }
          });

  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  updateStatusBar();
  resubscribeAll();
  follow_timer_.start();
}

MapPanel::~MapPanel() { unsubscribeAll(); }

void MapPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("MapDock");
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
  options.on_reset = [this]() {
    if (view_ != nullptr) {
      view_->recenter();
    }
  };
  options.show_settings = true;
  options.settings_checked = settingsVisible();
  options.on_settings_toggled = [this](bool visible) { onToggleSettings(visible); };
  options.on_expand = [this]() { emit panelExpandRequested(); };

  const PanelTitleBarTools tools =
      CreatePanelTitleBarTools(dock, callbacks, options);
  settings_button_ = tools.settings_button;
  expand_button_ = tools.expand_button;
  dock->setTitleBarTools(tools.widget);
}

MapPanelConfig MapPanel::config() const { return config_; }

void MapPanel::setConfig(const MapPanelConfig& config) {
  config_ = config;
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  resubscribeAll();
  updateStatusBar();
}

void MapPanel::cloneConfigFrom(const MapPanelConfig& config) { setConfig(config); }

void MapPanel::setSettingsVisible(bool visible) {
  if (settings_container_ != nullptr) {
    settings_container_->setVisible(visible);
  }
  syncSettingsToolState();
}

bool MapPanel::settingsVisible() const {
  return settings_container_ != nullptr && settings_container_->isVisible();
}

void MapPanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ != nullptr) {
    settings_button_->blockSignals(true);
    settings_button_->setChecked(checked);
    settings_button_->blockSignals(false);
  }
}

void MapPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ != nullptr) {
    expand_button_->blockSignals(true);
    expand_button_->setChecked(checked);
    expand_button_->blockSignals(false);
  }
}

QWidget* MapPanel::settingsWidgetForInspector() {
  return SettingsScrollForInspector(settings_scroll_);
}

void MapPanel::recallSettingsWidget() {
  RecallSettingsScrollToContainer(settings_scroll_, settings_container_);
}

void MapPanel::refreshSettingsChannels() {
  if (settings_widget_ != nullptr) {
    settings_widget_->refreshChannels();
  }
}

void MapPanel::handleChannelDrop(const QString& channel) {
  if (channel.isEmpty()) {
    return;
  }
  ensureTopicLayerForChannel(channel);
  if (settings_widget_ != nullptr) {
    config_ = settings_widget_->config();
  }
  resubscribeAll();
  updateStatusBar();
  emit configChanged();
}

void MapPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void MapPanel::dragEnterEvent(QDragEnterEvent* event) {
  if (event == nullptr) {
    return;
  }
  if (readDropPayload(event->mimeData(), nullptr) ||
      geoJsonPathFromMime(event->mimeData()).isEmpty() == false) {
    event->acceptProposedAction();
  }
}

void MapPanel::dragMoveEvent(QDragMoveEvent* event) {
  if (event == nullptr) {
    return;
  }
  if (readDropPayload(event->mimeData(), nullptr) ||
      geoJsonPathFromMime(event->mimeData()).isEmpty() == false) {
    event->acceptProposedAction();
  }
}

void MapPanel::dropEvent(QDropEvent* event) {
  if (event == nullptr) {
    return;
  }
  const QString geojson_path = geoJsonPathFromMime(event->mimeData());
  if (!geojson_path.isEmpty() && view_ != nullptr) {
    view_->loadGeoJsonFile(geojson_path);
    event->acceptProposedAction();
    return;
  }
  QString channel;
  if (!readDropPayload(event->mimeData(), &channel)) {
    return;
  }
  handleChannelDrop(channel);
  event->acceptProposedAction();
}

void MapPanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

void MapPanel::onFollowTick() { refreshViewport(); }

void MapPanel::onViewChanged(double latitude, double longitude, double zoom) {
  config_.center_latitude = latitude;
  config_.center_longitude = longitude;
  config_.zoom = zoom;
  updateStatusBar();
  const bool panning = view_ != nullptr && view_->isPanning();
  if (settings_widget_ != nullptr && !panning) {
    settings_widget_->blockSignals(true);
    settings_widget_->setConfig(config_);
    settings_widget_->blockSignals(false);
  }
  if (!panning) {
    emit configChanged();
  }
}

QString MapPanel::messageTypeForChannel(const QString& channel) const {
  if (manager_ == nullptr || channel.isEmpty()) {
    return {};
  }
  for (const integration::ChannelInfo& info : manager_->channels()) {
    if (QString::fromStdString(info.channel_name) == channel) {
      return QString::fromStdString(info.message_type);
    }
  }
  return {};
}

void MapPanel::resubscribeAll() {
  QSet<QString> desired_channels;
  for (const MapTopicLayerConfig& layer : config_.topic_layers) {
    if (!layer.enabled || layer.channel.isEmpty()) {
      continue;
    }
    desired_channels.insert(layer.channel);
  }

  for (auto it = subscriptions_.cbegin(); it != subscriptions_.cend();) {
    if (!desired_channels.contains(it.key())) {
      if (it.value() != 0) {
        integration::ChannelReaderRegistry::instance().unsubscribe(it.value());
      }
      subscribed_message_types_.remove(it.key());
      layer_store_.removeChannel(it.key());
      it = subscriptions_.erase(it);
    } else {
      ++it;
    }
  }

  for (const QString& channel : desired_channels) {
    MapTopicLayerConfig style;
    for (const MapTopicLayerConfig& layer : config_.topic_layers) {
      if (layer.channel == channel) {
        style = layer;
        break;
      }
    }
    if (subscriptions_.contains(channel)) {
      layer_store_.updateLayerStyle(channel, style);
      continue;
    }
    const std::string message_type = messageTypeForChannel(channel).toStdString();
    subscribed_message_types_.insert(channel, message_type);
    subscriptions_.insert(
        channel, integration::ChannelReaderRegistry::instance().subscribe(
                     channel.toStdString(), [this, channel](const std::string& payload) {
                       ingestChannelPayload(channel, payload);
                     }));
  }
  refreshViewport();
  updateStatusBar();
}

void MapPanel::unsubscribeAll() {
  for (auto it = subscriptions_.cbegin(); it != subscriptions_.cend(); ++it) {
    if (it.value() != 0) {
      integration::ChannelReaderRegistry::instance().unsubscribe(it.value());
    }
  }
  subscriptions_.clear();
  subscribed_message_types_.clear();
}

void MapPanel::ingestChannelPayload(const QString& channel,
                                    const std::string& payload) {
  const QString message_type = messageTypeForChannel(channel);
  if (!MapMessageIngest::SupportsMessageType(message_type)) {
    return;
  }

  MapTopicLayerConfig style;
  for (const MapTopicLayerConfig& layer : config_.topic_layers) {
    if (layer.channel == channel) {
      style = layer;
      break;
    }
  }

  QString error;
  const MapIngestResult result =
      MapMessageIngest::Ingest(message_type, payload, &error);
  if (!error.isEmpty() && result.points.isEmpty() && result.lines.isEmpty() &&
      result.polygons.isEmpty()) {
    return;
  }
  layer_store_.ingest(channel, style, result, nowNanoseconds());
  refreshViewport();
}

void MapPanel::refreshViewport() {
  if (view_ == nullptr) {
    return;
  }
  view_->setConfig(config_);
  view_->setLayers(layer_store_.snapshot(nowNanoseconds()));
  view_->setFollowTarget(
      layer_store_.followTarget(config_.follow_channel, nowNanoseconds()));
  view_->setGcsTarget(
      layer_store_.followTarget(config_.gcs_channel, nowNanoseconds()));
}

void MapPanel::applyConfigToUi() {
  refreshViewport();
  updateStatusBar();
}

void MapPanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ != nullptr) {
    settings_widget_->setConfig(config_);
  }
}

void MapPanel::syncSettingsToolState() {
  setSettingsButtonChecked(settingsVisible());
}

void MapPanel::updateStatusBar() {
  if (status_label_ == nullptr) {
    return;
  }
  int enabled_layers = 0;
  for (const MapTopicLayerConfig& layer : config_.topic_layers) {
    if (layer.enabled && !layer.channel.isEmpty()) {
      ++enabled_layers;
    }
  }
  QString text;
  if (qIsFinite(cursor_latitude_) && qIsFinite(cursor_longitude_)) {
    text = tr("%1°, %2°  ·  z %3")
               .arg(cursor_latitude_, 0, 'f', 5)
               .arg(cursor_longitude_, 0, 'f', 5)
               .arg(config_.zoom, 0, 'f', 1);
  } else {
    text = tr("%1°, %2°  ·  z %3")
               .arg(config_.center_latitude, 0, 'f', 5)
               .arg(config_.center_longitude, 0, 'f', 5)
               .arg(config_.zoom, 0, 'f', 1);
  }
  if (enabled_layers > 0) {
    text += tr("  ·  %1 layer(s)").arg(enabled_layers);
  }
  if (!config_.waypoints.isEmpty() || !config_.geofence.isEmpty() ||
      !config_.rally_points.isEmpty()) {
    text += tr("  ·  plan %1w/%2f/%3r")
                .arg(config_.waypoints.size())
                .arg(config_.geofence.size())
                .arg(config_.rally_points.size());
  }
  if (!map_status_extra_.isEmpty()) {
    text += QStringLiteral("  ·  ") + map_status_extra_;
  }
  status_label_->setText(text);
}

QString MapPanel::geoJsonPathFromMime(const QMimeData* mime) const {
  if (mime == nullptr || !mime->hasUrls()) {
    return {};
  }
  for (const QUrl& url : mime->urls()) {
    const QString path = url.toLocalFile();
    if (path.endsWith(QStringLiteral(".geojson"), Qt::CaseInsensitive) ||
        path.endsWith(QStringLiteral(".json"), Qt::CaseInsensitive)) {
      return path;
    }
  }
  return {};
}

bool MapPanel::readDropPayload(const QMimeData* mime, QString* channel) const {
  if (mime == nullptr) {
    return false;
  }
  plot::PlotSeriesDragPayload payload;
  if (!plot::ReadPlotSeriesDragPayload(mime, &payload) || payload.channel.isEmpty()) {
    return false;
  }
  const QString message_type = messageTypeForChannel(payload.channel);
  if (!MapMessageIngest::SupportsMessageType(message_type)) {
    return false;
  }
  if (channel != nullptr) {
    *channel = payload.channel;
  }
  return true;
}

void MapPanel::ensureTopicLayerForChannel(const QString& channel) {
  for (const MapTopicLayerConfig& layer : config_.topic_layers) {
    if (layer.channel == channel) {
      return;
    }
  }
  MapTopicLayerConfig layer;
  layer.channel = channel;
  layer.color = QColor(255, 90, 60);
  config_.topic_layers.push_back(layer);
  if (settings_widget_ != nullptr) {
    settings_widget_->setConfig(config_);
  }
}

quint64 MapPanel::nowNanoseconds() const {
  return static_cast<quint64>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}  // namespace map
}  // namespace autoviz

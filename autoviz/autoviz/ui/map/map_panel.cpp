/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_panel.hpp"

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMimeData>
#include <QScrollArea>
#include <QSet>
#include <QToolButton>
#include <QVBoxLayout>

#include <chrono>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/map/map_message_ingest.hpp"
#include "autoviz/ui/map/map_settings_widget.hpp"
#include "autoviz/ui/map/map_viewport_widget.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"

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
  settings_scroll_->setWidgetResizable(true);
  settings_scroll_->setFrameShape(QFrame::NoFrame);
  settings_scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  settings_widget_ = new MapSettingsWidget(manager_, settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
  settings_layout->addWidget(settings_scroll_);

  auto* toolbar = new QFrame(this);
  ApplyPanelToolbarChrome(toolbar);
  auto* toolbar_layout = new QHBoxLayout(toolbar);
  toolbar_layout->setContentsMargins(6, 4, 6, 4);
  status_label_ = new QLabel(toolbar);
  status_label_->setStyleSheet(PanelStatusLabelStyle());
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  toolbar_layout->addWidget(status_label_, 1);
  root->addWidget(toolbar);

  view_ = new MapViewportWidget(this);
  root->addWidget(view_, 1);

  follow_timer_.setInterval(100);
  connect(&follow_timer_, &QTimer::timeout, this, &MapPanel::onFollowTick);

  connect(settings_widget_, &MapSettingsWidget::configChanged, this, [this]() {
    config_ = settings_widget_->config();
    applyConfigToUi();
    resubscribeAll();
    emit configChanged();
  });
  connect(view_, &MapViewportWidget::viewChanged, this, &MapPanel::onViewChanged);

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
  options.show_settings = true;
  options.settings_checked = settingsVisible();
  options.on_settings_toggled = [this](bool visible) { onToggleSettings(visible); };
  options.on_expand = [this]() { emit panelExpandRequested(); };

  const PanelTitleBarTools tools =
      CreateRvizPanelTitleBarTools(dock, callbacks, options);
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
  if (event != nullptr && readDropPayload(event->mimeData(), nullptr)) {
    event->acceptProposedAction();
  }
}

void MapPanel::dragMoveEvent(QDragMoveEvent* event) {
  if (event != nullptr && readDropPayload(event->mimeData(), nullptr)) {
    event->acceptProposedAction();
  }
}

void MapPanel::dropEvent(QDropEvent* event) {
  if (event == nullptr) {
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
  if (settings_widget_ != nullptr) {
    settings_widget_->blockSignals(true);
    settings_widget_->setConfig(config_);
    settings_widget_->blockSignals(false);
  }
  emit configChanged();
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
  if (enabled_layers == 0) {
    status_label_->setText(tr("Drop a geo channel from Channels"));
    return;
  }
  status_label_->setText(tr("%1 topic layer(s)").arg(enabled_layers));
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

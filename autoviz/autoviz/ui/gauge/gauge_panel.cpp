/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/gauge/gauge_panel.hpp"

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMimeData>
#include <QScrollArea>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/gauge/gauge_settings_widget.hpp"
#include "autoviz/ui/gauge/gauge_view_widget.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/plot/plot_field_extractor.hpp"
#include "autoviz/ui/plot/plot_field_path.hpp"
#include "autoviz/variables/variable_path_utils.hpp"

namespace autoviz {
namespace gauge {
namespace {


}  // namespace

GaugePanel::GaugePanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultGaugePanelConfig()), QWidget(parent) {
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
  settings_widget_ = new GaugeSettingsWidget(manager_, settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
  settings_layout->addWidget(settings_scroll_);

  auto* toolbar = new QFrame(this);
  ApplyPanelToolbarChrome(toolbar);
  auto* toolbar_layout = new QHBoxLayout(toolbar);
  toolbar_layout->setContentsMargins(6, 2, 6, 2);
  status_label_ = new QLabel(toolbar);
  status_label_->setStyleSheet(PanelStatusLabelStyle());
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  toolbar_layout->addWidget(status_label_, 1);
  root->addWidget(toolbar);

  view_ = new GaugeViewWidget(this);
  root->addWidget(view_, 1);

  connect(settings_widget_, &GaugeSettingsWidget::configChanged, this, [this]() {
    const QString previous_channel = config_.channel;
    config_ = settings_widget_->config();
    applyConfigToUi();
    if (config_.channel != previous_channel) {
      resubscribeChannel();
    }
    emit configChanged();
  });

  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  updateStatusBar();
  resubscribeChannel();
}

GaugePanel::~GaugePanel() { unsubscribeChannel(); }

void GaugePanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("GaugeDock");
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

GaugePanelConfig GaugePanel::config() const { return config_; }

void GaugePanel::setConfig(const GaugePanelConfig& config) {
  config_ = config;
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  resubscribeChannel();
}

void GaugePanel::cloneConfigFrom(const GaugePanelConfig& config) {
  setConfig(config);
}

void GaugePanel::setSettingsVisible(bool visible) {
  if (settings_container_ != nullptr) {
    settings_container_->setVisible(visible);
  }
  syncSettingsToolState();
}

bool GaugePanel::settingsVisible() const {
  return settings_container_ != nullptr && settings_container_->isVisible();
}

void GaugePanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ != nullptr) {
    settings_button_->blockSignals(true);
    settings_button_->setChecked(checked);
    settings_button_->blockSignals(false);
  }
}

void GaugePanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ != nullptr) {
    expand_button_->blockSignals(true);
    expand_button_->setChecked(checked);
    expand_button_->blockSignals(false);
  }
}

QWidget* GaugePanel::settingsWidgetForInspector() {
  return SettingsScrollForInspector(settings_scroll_);
}

void GaugePanel::recallSettingsWidget() {
  RecallSettingsScrollToContainer(settings_scroll_, settings_container_);
}

void GaugePanel::refreshSettingsChannels() {
  if (settings_widget_ != nullptr) {
    settings_widget_->refreshChannels();
  }
}

void GaugePanel::handleFieldDrop(const QString& channel, const QString& field_path) {
  if (channel.isEmpty() || field_path.isEmpty()) {
    return;
  }
  config_.channel = channel;
  config_.field_path = field_path;
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  resubscribeChannel();
  emit configChanged();
  emit activated();
}

void GaugePanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

bool GaugePanel::readDropPayload(const QMimeData* mime, QString* channel,
                                 QString* field_path) const {
  if (mime == nullptr || channel == nullptr || field_path == nullptr) {
    return false;
  }
  plot::PlotSeriesDragPayload payload;
  if (!plot::ReadPlotSeriesDragPayload(mime, &payload) || payload.channel.isEmpty() ||
      payload.field_path.isEmpty()) {
    return false;
  }
  *channel = payload.channel;
  *field_path = payload.field_path;
  return true;
}

void GaugePanel::dragEnterEvent(QDragEnterEvent* event) {
  if (event == nullptr) {
    return;
  }
  QString channel;
  QString field_path;
  if (readDropPayload(event->mimeData(), &channel, &field_path)) {
    event->acceptProposedAction();
    emit activated();
  }
}

void GaugePanel::dragMoveEvent(QDragMoveEvent* event) {
  if (event == nullptr) {
    return;
  }
  QString channel;
  QString field_path;
  if (readDropPayload(event->mimeData(), &channel, &field_path)) {
    event->acceptProposedAction();
  }
}

void GaugePanel::dropEvent(QDropEvent* event) {
  if (event == nullptr) {
    return;
  }
  QString channel;
  QString field_path;
  if (!readDropPayload(event->mimeData(), &channel, &field_path)) {
    return;
  }
  handleFieldDrop(channel, field_path);
  event->acceptProposedAction();
}

void GaugePanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

QString GaugePanel::messageTypeForChannel(const QString& channel) const {
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

void GaugePanel::resubscribeChannel() {
  unsubscribeChannel();
  if (config_.channel.isEmpty()) {
    view_->setErrorText(tr("Drop a numeric field from Channels"));
    updateStatusBar();
    return;
  }
  subscribed_message_type_ = messageTypeForChannel(config_.channel).toStdString();
  subscription_id_ = integration::ChannelReaderRegistry::instance().subscribe(
      config_.channel.toStdString(),
      [this](const std::string& payload) { onChannelPayload(payload); });
}

void GaugePanel::unsubscribeChannel() {
  if (subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(subscription_id_);
    subscription_id_ = 0;
  }
  subscribed_message_type_.clear();
}

void GaugePanel::onChannelPayload(const std::string& payload) {
  ingestPayload(payload);
}

void GaugePanel::refreshFromVariables() {
  if (!last_payload_.empty()) {
    ingestPayload(last_payload_);
  }
}

void GaugePanel::ingestPayload(const std::string& payload) {
  last_payload_ = payload;
  if (config_.field_path.isEmpty()) {
    view_->setErrorText(tr("Configure a numeric field path"));
    updateStatusBar();
    return;
  }

  const plot::ParsedFieldPath parsed = plot::ParseFieldPath(
      plot::ResolvePlotFieldPath(config_.field_path, &manager_->variableStore()));
  if (!parsed.modifiers.isEmpty()) {
    view_->setErrorText(
        tr("Plot modifiers (e.g. .@derivative) are not supported on Gauge panels"));
    updateStatusBar();
    return;
  }

  if (subscribed_message_type_.empty()) {
    view_->setErrorText(tr("Unknown message type for channel"));
    updateStatusBar();
    return;
  }

  const std::optional<double> value = plot::PlotFieldExtractor::instance().extractNumeric(
      subscribed_message_type_, payload, parsed.base_path.toStdString());
  if (!value.has_value()) {
    view_->setErrorText(tr("Field is not numeric or unavailable"));
    updateStatusBar();
    return;
  }

  view_->setConfig(config_);
  view_->setValue(*value, true);
  updateStatusBar();
}

void GaugePanel::applyConfigToUi() {
  if (view_ != nullptr) {
    view_->setConfig(config_);
  }
  updateStatusBar();
}

void GaugePanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ != nullptr) {
    settings_widget_->setConfig(config_);
  }
}

void GaugePanel::syncSettingsToolState() {
  setSettingsButtonChecked(settingsVisible());
}

void GaugePanel::updateStatusBar() {
  if (status_label_ == nullptr) {
    return;
  }
  if (config_.channel.isEmpty() || config_.field_path.isEmpty()) {
    status_label_->setText(tr("Drop a numeric field from Channels"));
    return;
  }
  status_label_->setText(
      QStringLiteral("%1 · %2").arg(config_.channel, config_.field_path));
}

}  // namespace gauge
}  // namespace autoviz

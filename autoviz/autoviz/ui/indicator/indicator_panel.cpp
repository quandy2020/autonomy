/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/indicator/indicator_panel.hpp"

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
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/indicator/indicator_field_extractor.hpp"
#include "autoviz/ui/indicator/indicator_rule_engine.hpp"
#include "autoviz/ui/indicator/indicator_settings_widget.hpp"
#include "autoviz/ui/indicator/indicator_view_widget.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/plot/plot_field_path.hpp"

namespace autoviz {
namespace indicator {
namespace {

QToolButton* AddTitleToolButton(QWidget* parent, const QIcon& icon,
                                const QString& tip, bool checkable = false) {
  return CreatePlotTitleToolButton(parent, icon, tip, checkable);
}

}  // namespace

IndicatorPanel::IndicatorPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultIndicatorPanelConfig()), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(QStringLiteral("IndicatorPanel { background: palette(window); }"));

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
  settings_widget_ = new IndicatorSettingsWidget(manager_, settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
  settings_layout->addWidget(settings_scroll_);

  auto* toolbar = new QFrame(this);
  toolbar->setStyleSheet(
      QStringLiteral(
          "QFrame {"
          "  background: palette(base);"
          "  border-bottom: 1px solid palette(midlight);"
          "}"));
  auto* toolbar_layout = new QHBoxLayout(toolbar);
  toolbar_layout->setContentsMargins(6, 4, 6, 4);
  status_label_ = new QLabel(toolbar);
  status_label_->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 10px;"));
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  toolbar_layout->addWidget(status_label_, 1);
  root->addWidget(toolbar);

  view_ = new IndicatorViewWidget(this);
  root->addWidget(view_, 1);

  connect(settings_widget_, &IndicatorSettingsWidget::configChanged, this, [this]() {
    const QString previous_channel = config_.channel;
    config_ = settings_widget_->config();
    applyConfigToUi();
    if (last_field_value_.has_value()) {
      applyMatchToView(*last_field_value_);
    }
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

IndicatorPanel::~IndicatorPanel() { unsubscribeChannel(); }

void IndicatorPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  auto* tools = new QWidget(dock);
  tools->setStyleSheet(PlotTitleToolsStyleSheet());
  auto* layout = new QHBoxLayout(tools);
  layout->setContentsMargins(0, 0, 4, 0);
  layout->setSpacing(0);
  layout->addWidget(CreateTitleSeparator(tools));

  auto* split_right = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_split_right.svg")),
      tr("Split right"));
  layout->addWidget(split_right);
  connect(split_right, &QToolButton::clicked, this, [this]() {
    emit panelSplitRequested(Qt::Horizontal);
  });

  auto* split_down = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_split_down.svg")),
      tr("Split down"));
  layout->addWidget(split_down);
  connect(split_down, &QToolButton::clicked, this, [this]() {
    emit panelSplitRequested(Qt::Vertical);
  });

  auto* expand = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_fullscreen.svg")),
      tr("Expand"), true);
  expand_button_ = expand;
  layout->addWidget(expand);
  connect(expand, &QToolButton::clicked, this, [this]() { emit panelExpandRequested(); });

  settings_button_ = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_settings.svg")),
      tr("Settings"), true);
  layout->addWidget(settings_button_);
  connect(settings_button_, &QToolButton::toggled, this, &IndicatorPanel::onToggleSettings);

  auto* more_button = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_more.svg")),
      tr("More"));
  more_button->setPopupMode(QToolButton::InstantPopup);
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("IndicatorDock");
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.split = [this](Qt::Orientation orientation) {
    emit panelSplitRequested(orientation);
  };
  callbacks.expand = [this]() { emit panelExpandRequested(); };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };
  more_button->setMenu(CreatePanelContextMenu(more_button, callbacks));
  layout->addWidget(more_button);

  dock->setTitleBarTools(tools);
}

IndicatorPanelConfig IndicatorPanel::config() const { return config_; }

void IndicatorPanel::setConfig(const IndicatorPanelConfig& config) {
  config_ = config;
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  resubscribeChannel();
}

void IndicatorPanel::cloneConfigFrom(const IndicatorPanelConfig& config) {
  setConfig(config);
}

void IndicatorPanel::setSettingsVisible(bool visible) {
  if (settings_container_ != nullptr) {
    settings_container_->setVisible(visible);
  }
  syncSettingsToolState();
}

bool IndicatorPanel::settingsVisible() const {
  return settings_container_ != nullptr && settings_container_->isVisible();
}

void IndicatorPanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ != nullptr) {
    settings_button_->blockSignals(true);
    settings_button_->setChecked(checked);
    settings_button_->blockSignals(false);
  }
}

void IndicatorPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ != nullptr) {
    expand_button_->blockSignals(true);
    expand_button_->setChecked(checked);
    expand_button_->blockSignals(false);
  }
}

QWidget* IndicatorPanel::settingsWidgetForInspector() {
  if (settings_widget_ == nullptr) {
    return nullptr;
  }
  settings_widget_->setParent(nullptr);
  return settings_widget_;
}

void IndicatorPanel::recallSettingsWidget() {
  if (settings_widget_ == nullptr || settings_scroll_ == nullptr) {
    return;
  }
  settings_widget_->setParent(settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
}

void IndicatorPanel::refreshSettingsChannels() {
  if (settings_widget_ != nullptr) {
    settings_widget_->refreshChannels();
  }
}

void IndicatorPanel::handleFieldDrop(const QString& channel, const QString& field_path) {
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

void IndicatorPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

bool IndicatorPanel::readDropPayload(const QMimeData* mime, QString* channel,
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

void IndicatorPanel::dragEnterEvent(QDragEnterEvent* event) {
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

void IndicatorPanel::dragMoveEvent(QDragMoveEvent* event) {
  if (event == nullptr) {
    return;
  }
  QString channel;
  QString field_path;
  if (readDropPayload(event->mimeData(), &channel, &field_path)) {
    event->acceptProposedAction();
  }
}

void IndicatorPanel::dropEvent(QDropEvent* event) {
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

void IndicatorPanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

QString IndicatorPanel::messageTypeForChannel(const QString& channel) const {
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

void IndicatorPanel::resubscribeChannel() {
  unsubscribeChannel();
  last_field_value_.reset();
  if (config_.channel.isEmpty()) {
    view_->setErrorText(tr("Drop a scalar field from Topics"));
    updateStatusBar();
    return;
  }
  subscribed_message_type_ = messageTypeForChannel(config_.channel).toStdString();
  subscription_id_ = integration::ChannelReaderRegistry::instance().subscribe(
      config_.channel.toStdString(),
      [this](const std::string& payload) { onChannelPayload(payload); });
}

void IndicatorPanel::unsubscribeChannel() {
  if (subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(subscription_id_);
    subscription_id_ = 0;
  }
  subscribed_message_type_.clear();
}

void IndicatorPanel::onChannelPayload(const std::string& payload) {
  ingestPayload(payload);
}

void IndicatorPanel::applyMatchToView(const IndicatorFieldValue& value) {
  const IndicatorMatchResult match = IndicatorRuleEngine::Evaluate(value, config_.rules);
  view_->setConfig(config_);
  view_->setMatchResult(match, true);
}

void IndicatorPanel::ingestPayload(const std::string& payload) {
  if (config_.field_path.isEmpty()) {
    view_->setErrorText(tr("Configure a field path"));
    updateStatusBar();
    return;
  }

  const plot::ParsedFieldPath parsed = plot::ParseFieldPath(config_.field_path);
  if (!parsed.modifiers.isEmpty()) {
    view_->setErrorText(
        tr("Plot modifiers (e.g. .@derivative) are not supported on Indicator panels"));
    updateStatusBar();
    return;
  }

  if (subscribed_message_type_.empty()) {
    view_->setErrorText(tr("Unknown message type for channel"));
    updateStatusBar();
    return;
  }

  const std::optional<IndicatorFieldValue> value =
      IndicatorFieldExtractor::instance().extract(
          subscribed_message_type_, payload, parsed.base_path.toStdString());
  if (!value.has_value()) {
    view_->setErrorText(tr("Field is not a scalar value or unavailable"));
    updateStatusBar();
    return;
  }

  last_field_value_ = *value;
  applyMatchToView(*value);
  updateStatusBar();
}

void IndicatorPanel::applyConfigToUi() {
  if (view_ != nullptr) {
    view_->setConfig(config_);
  }
  updateStatusBar();
}

void IndicatorPanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ != nullptr) {
    settings_widget_->setConfig(config_);
  }
}

void IndicatorPanel::syncSettingsToolState() {
  setSettingsButtonChecked(settingsVisible());
}

void IndicatorPanel::updateStatusBar() {
  if (status_label_ == nullptr) {
    return;
  }
  if (config_.channel.isEmpty() || config_.field_path.isEmpty()) {
    status_label_->setText(tr("Drop a scalar field from Topics"));
    return;
  }
  status_label_->setText(
      QStringLiteral("%1 · %2").arg(config_.channel, config_.field_path));
}

}  // namespace indicator
}  // namespace autoviz

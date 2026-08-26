/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/log/log_panel.hpp"

#include <QCheckBox>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QScrollArea>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/log/log_hub.hpp"
#include "autoviz/ui/log/log_parser.hpp"
#include "autoviz/ui/log/log_settings_widget.hpp"
#include "autoviz/ui/log/log_view_widget.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"

namespace autoviz {
namespace log_panel {
namespace {

constexpr char kBg[] = "#f8f9fb";
constexpr char kSurface[] = "#ffffff";
constexpr char kBorder[] = "#cbd5e1";
constexpr char kText[] = "#1e293b";
constexpr char kTextMuted[] = "#64748b";
constexpr char kAccent[] = "#0891b2";

double TimestampSec(const LogEntry& entry) {
  return static_cast<double>(entry.timestamp_ns) / 1e9;
}

}  // namespace

LogPanel::LogPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultLogPanelConfig()), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  ApplyPanelShell(this);
  setObjectName(QStringLiteral("LogPanelContent"));
  applyChromeStyles();

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
  settings_widget_ = new LogSettingsWidget(manager_, settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
  settings_layout->addWidget(settings_scroll_);

  auto* toolbar = new QFrame(this);
  toolbar->setObjectName(QStringLiteral("LogToolbar"));
  auto* toolbar_layout = new QHBoxLayout(toolbar);
  toolbar_layout->setContentsMargins(10, 8, 10, 8);
  toolbar_layout->setSpacing(8);

  auto* filter_icon = new QLabel(QStringLiteral("⌕"), toolbar);
  filter_icon->setStyleSheet(
      QStringLiteral("color: %1; font-size: 14px;").arg(QLatin1String(kTextMuted)));
  filter_icon->setFixedWidth(16);
  toolbar_layout->addWidget(filter_icon);

  search_edit_ = new QLineEdit(toolbar);
  search_edit_->setPlaceholderText(tr("Filter logs"));
  search_edit_->setClearButtonEnabled(false);
  search_edit_->setStyleSheet(QStringLiteral(
      "QLineEdit {"
      "  background: %1; color: %2;"
      "  border: 1px solid %3; border-radius: 8px;"
      "  padding: 6px 10px; min-height: 26px;"
      "}"
      "QLineEdit:focus { border-color: %4; }")
                                  .arg(QLatin1String(kSurface), QLatin1String(kText),
                                       QLatin1String(kBorder), QLatin1String(kAccent)));
  toolbar_layout->addWidget(search_edit_, 1);

  search_clear_button_ = new QToolButton(toolbar);
  search_clear_button_->setText(QStringLiteral("×"));
  search_clear_button_->setToolTip(tr("Clear search"));
  search_clear_button_->setCursor(Qt::PointingHandCursor);
  search_clear_button_->setVisible(false);
  search_clear_button_->setFixedSize(22, 22);
  search_clear_button_->setStyleSheet(QStringLiteral(
      "QToolButton {"
      "  color: %1; background: rgba(8,145,178,0.10);"
      "  border: 1px solid rgba(8,145,178,0.28); border-radius: 11px;"
      "}"
      "QToolButton:hover { background: rgba(8,145,178,0.18); }")
                                          .arg(QLatin1String(kAccent)));
  toolbar_layout->addWidget(search_clear_button_);

  status_label_ = new QLabel(toolbar);
  status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  status_label_->setStyleSheet(QStringLiteral(
      "color: %1; background: rgba(8,145,178,0.10);"
      "border: 1px solid rgba(8,145,178,0.28); border-radius: 10px;"
      "padding: 3px 10px; font-size: 11px; font-weight: 700;")
                                   .arg(QLatin1String(kAccent)));
  toolbar_layout->addWidget(status_label_);
  root->addWidget(toolbar);

  auto* list_card = new QFrame(this);
  list_card->setObjectName(QStringLiteral("LogListCard"));
  auto* list_card_layout = new QVBoxLayout(list_card);
  list_card_layout->setContentsMargins(1, 1, 1, 1);
  list_card_layout->setSpacing(0);
  view_ = new LogViewWidget(list_card);
  list_card_layout->addWidget(view_, 1);
  root->addWidget(list_card, 1);

  follow_timer_ = new QTimer(this);
  follow_timer_->setInterval(250);
  connect(follow_timer_, &QTimer::timeout, this, &LogPanel::onFollowTick);

  channel_refresh_timer_ = new QTimer(this);
  channel_refresh_timer_->setInterval(2000);
  connect(channel_refresh_timer_, &QTimer::timeout, this,
          &LogPanel::refreshLogChannelSubscriptions);

  connect(settings_widget_, &LogSettingsWidget::configChanged, this, [this]() {
    const QString previous_topic = config_.topic;
    const bool previous_auto = config_.auto_subscribe_log_channels;
    config_ = settings_widget_->config();
    applyConfigToUi();
    updateNamespaceUi();
    if (config_.topic != previous_topic ||
        config_.auto_subscribe_log_channels != previous_auto) {
      resubscribeTopic();
    }
    emit configChanged();
  });
  connect(search_edit_, &QLineEdit::textChanged, this, &LogPanel::onSearchChanged);
  connect(search_clear_button_, &QToolButton::clicked, this, [this]() {
    search_edit_->clear();
  });
  connect(view_, &LogViewWidget::entryClicked, this, &LogPanel::onEntryClicked);
  connect(view_, &LogViewWidget::entryHovered, this, &LogPanel::onEntryHovered);
  connect(view_, &LogViewWidget::statsChanged, this, &LogPanel::onStatsChanged);
  connect(&LogHub::instance(), &LogHub::logAppended, this, &LogPanel::onHubLogAppended);

  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  for (const LogEntry& entry : LogHub::instance().recentEntries()) {
    ingestEntry(entry);
  }
  resubscribeTopic();
  channel_refresh_timer_->start();
  updateStatusBar(view_->visibleEntryCount(), view_->totalEntryCount(),
                  view_->pinnedToBottom());
  follow_timer_->start();
}

LogPanel::~LogPanel() { unsubscribeTopic(); }

void LogPanel::applyChromeStyles() {
  setStyleSheet(QStringLiteral(
      "QWidget#LogPanelContent {"
      "  background: %1; color: %2;"
      "}"
      "QFrame#LogToolbar {"
      "  background: %3;"
      "  border-bottom: 1px solid %4;"
      "}"
      "QFrame#LogListCard {"
      "  background: %3;"
      "  border: none;"
      "}")
                    .arg(QLatin1String(kBg), QLatin1String(kText),
                         QLatin1String(kSurface), QLatin1String(kBorder)));
}

void LogPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("LogDock");
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
  options.settings_checked = config_.settings_visible;
  options.on_settings_toggled = [this](bool visible) { onToggleSettings(visible); };
  options.on_expand = [this]() { emit panelExpandRequested(); };

  const PanelTitleBarTools tools =
      CreateRvizPanelTitleBarTools(dock, callbacks, options);
  settings_button_ = tools.settings_button;
  expand_button_ = tools.expand_button;
  dock->setTitleBarTools(tools.widget);
}

LogPanelConfig LogPanel::config() const { return config_; }

void LogPanel::setConfig(const LogPanelConfig& config) {
  config_ = config;
  resubscribeTopic();
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  syncSettingsToolState();
}

void LogPanel::cloneConfigFrom(const LogPanelConfig& config) {
  setConfig(config);
}

void LogPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ == nullptr) {
    return;
  }
  expand_button_->blockSignals(true);
  expand_button_->setChecked(checked);
  expand_button_->blockSignals(false);
}

void LogPanel::applyConfigToUi() {
  if (view_ == nullptr) {
    return;
  }
  view_->setFontSize(config_.font_size);
  view_->setMinLevel(config_.min_level);
  view_->setSearchTerms(parseSearchTerms(search_edit_->text()));
  view_->setEnabledNamespaces(enabledNamespacesFromSettings());
}

void LogPanel::refreshSettingsChannels() {
  if (settings_widget_ != nullptr) {
    settings_widget_->setConfig(config_);
  }
  refreshLogChannelSubscriptions();
}

QStringList LogPanel::parseSearchTerms(const QString& text) const {
  QStringList terms;
  for (const QString& part : text.split(QLatin1Char(','), Qt::SkipEmptyParts)) {
    const QString trimmed = part.trimmed();
    if (!trimmed.isEmpty()) {
      terms.push_back(trimmed);
    }
  }
  return terms;
}

QSet<QString> LogPanel::enabledNamespacesFromSettings() const {
  QSet<QString> enabled;
  if (settings_widget_ == nullptr) {
    return enabled;
  }
  const auto checks = settings_widget_->findChildren<QCheckBox*>();
  for (QCheckBox* check : checks) {
    const QVariant name = check->property("namespaceName");
    if (!name.isValid()) {
      continue;
    }
    if (check->isChecked()) {
      enabled.insert(name.toString());
    }
  }
  return enabled;
}

void LogPanel::updateNamespaceUi() {
  if (settings_widget_ != nullptr) {
    settings_widget_->setKnownNamespaces(known_namespaces_);
  }
  view_->setEnabledNamespaces(enabledNamespacesFromSettings());
}

void LogPanel::ingestEntry(const LogEntry& entry) {
  if (!entry.name.isEmpty()) {
    known_namespaces_.insert(entry.name);
    updateNamespaceUi();
  }
  view_->addEntry(entry);
}

void LogPanel::onHubLogAppended(const LogEntry& entry) {
  if (entry.source == QLatin1String("glog") && !config_.capture_glog) {
    return;
  }
  ingestEntry(entry);
}

void LogPanel::onChannelPayload(const QString& channel,
                                const std::string& payload) {
  std::string message_type = "foxglove.Log";
  if (manager_ != nullptr) {
    const std::string channel_std = channel.toStdString();
    for (const integration::ChannelInfo& info : manager_->channels()) {
      if (info.channel_name == channel_std) {
        message_type = info.message_type;
        break;
      }
    }
  }
  LogEntry entry =
      logEntryFromPayload(message_type, payload, channel);
  if (entry.message.isEmpty()) {
    return;
  }
  if (entry.source.isEmpty()) {
    entry.source = channel;
  }
  LogHub::instance().append(std::move(entry));
}

QStringList LogPanel::discoverLogChannels() const {
  QStringList channels;
  QSet<QString> seen;
  auto add = [&](const QString& name) {
    const QString trimmed = name.trimmed();
    if (trimmed.isEmpty() || seen.contains(trimmed)) {
      return;
    }
    seen.insert(trimmed);
    channels.push_back(trimmed);
  };

  add(config_.topic);
  add(QStringLiteral("/rosout"));
  if (config_.auto_subscribe_log_channels && manager_ != nullptr) {
    for (const integration::ChannelInfo& info : manager_->channels()) {
      if (isLogMessageType(info.message_type)) {
        add(QString::fromStdString(info.channel_name));
      }
    }
  }
  return channels;
}

void LogPanel::refreshLogChannelSubscriptions() {
  const QStringList wanted = discoverLogChannels();
  if (wanted.size() == subscribed_channels_.size()) {
    bool same = true;
    for (const QString& channel : wanted) {
      if (!subscribed_channels_.contains(channel)) {
        same = false;
        break;
      }
    }
    if (same) {
      return;
    }
  }
  resubscribeTopic();
}

void LogPanel::resubscribeTopic() {
  unsubscribeTopic();
  const QStringList channels = discoverLogChannels();
  for (const QString& channel : channels) {
    const auto id = integration::ChannelReaderRegistry::instance().subscribe(
        channel.toStdString(),
        [this, channel](const std::string& payload) {
          onChannelPayload(channel, payload);
        });
    if (id != 0) {
      topic_subscription_ids_.push_back(id);
      subscribed_channels_.insert(channel);
    }
  }
}

void LogPanel::unsubscribeTopic() {
  for (const auto id : topic_subscription_ids_) {
    if (id != 0) {
      integration::ChannelReaderRegistry::instance().unsubscribe(id);
    }
  }
  topic_subscription_ids_.clear();
  subscribed_channels_.clear();
}

void LogPanel::onSearchChanged(const QString& text) {
  search_clear_button_->setVisible(!text.trimmed().isEmpty());
  view_->setSearchTerms(parseSearchTerms(text));
}

void LogPanel::onStatsChanged(int visible_count, int total_count, bool pinned) {
  updateStatusBar(visible_count, total_count, pinned);
}

void LogPanel::updateStatusBar(int visible_count, int total_count, bool pinned) {
  if (status_label_ == nullptr) {
    return;
  }
  QString text;
  if (total_count == 0) {
    text = tr("0");
  } else if (visible_count == total_count) {
    text = QString::number(visible_count);
  } else {
    text = tr("%1/%2").arg(visible_count).arg(total_count);
  }
  if (pinned && visible_count > 0) {
    text += tr(" ↓");
  }
  status_label_->setText(text);
}

void LogPanel::onEntryClicked(const LogEntry& entry) {
  if (manager_ == nullptr || entry.timestamp_ns <= 0) {
    return;
  }
  manager_->playback().seekTo(TimestampSec(entry));
}

void LogPanel::onEntryHovered(const LogEntry& /*entry*/) {}

void LogPanel::onFollowTick() {
  if (!config_.follow_playback || manager_ == nullptr) {
    return;
  }
  view_->followTimestamp(manager_->simTimeSec());
}

void LogPanel::setSettingsVisible(bool visible) {
  config_.settings_visible = visible;
  syncSettingsToolState();
}

bool LogPanel::settingsVisible() const { return config_.settings_visible; }

void LogPanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ == nullptr) {
    return;
  }
  settings_button_->blockSignals(true);
  settings_button_->setChecked(checked);
  settings_button_->blockSignals(false);
}

QWidget* LogPanel::settingsWidgetForInspector() {
  return SettingsScrollForInspector(settings_scroll_);
}

void LogPanel::recallSettingsWidget() {
  RecallSettingsScrollToContainer(settings_scroll_, settings_container_);
}

void LogPanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ == nullptr) {
    return;
  }
  settings_widget_->blockSignals(true);
  settings_widget_->setConfig(config_);
  settings_widget_->setKnownNamespaces(known_namespaces_);
  settings_widget_->blockSignals(false);
}

void LogPanel::syncSettingsToolState() {
  setSettingsButtonChecked(config_.settings_visible);
}

void LogPanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

void LogPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

}  // namespace log_panel
}  // namespace autoviz

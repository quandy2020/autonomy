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
#include "autoviz/ui/panel_title_tools.hpp"

namespace autoviz {
namespace log_panel {
namespace {

QToolButton* AddTitleToolButton(QWidget* parent, const QIcon& icon,
                                const QString& tip, bool checkable = false) {
  return CreatePlotTitleToolButton(parent, icon, tip, checkable);
}

double TimestampSec(const LogEntry& entry) {
  return static_cast<double>(entry.timestamp_ns) / 1e9;
}

}  // namespace

LogPanel::LogPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultLogPanelConfig()), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(QStringLiteral("LogPanel { background: palette(window); }"));

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
  toolbar->setStyleSheet(
      QStringLiteral(
          "QFrame {"
          "  background: palette(base);"
          "  border-bottom: 1px solid palette(midlight);"
          "}"));
  auto* toolbar_layout = new QHBoxLayout(toolbar);
  toolbar_layout->setContentsMargins(6, 4, 6, 4);
  toolbar_layout->setSpacing(4);

  search_edit_ = new QLineEdit(toolbar);
  search_edit_->setPlaceholderText(tr("Filter logs"));
  search_edit_->setClearButtonEnabled(false);
  search_edit_->setStyleSheet(
      QStringLiteral(
          "QLineEdit {"
          "  background: palette(base);"
          "  border: 1px solid palette(mid);"
          "  border-radius: 3px;"
          "  padding: 2px 6px;"
          "  color: palette(text);"
          "  min-height: 18px;"
          "}"
          "QLineEdit:focus { border-color: palette(highlight); }"));
  toolbar_layout->addWidget(search_edit_, 1);

  search_clear_button_ = new QToolButton(toolbar);
  search_clear_button_->setText(QStringLiteral("×"));
  search_clear_button_->setToolTip(tr("Clear search"));
  search_clear_button_->setCursor(Qt::PointingHandCursor);
  search_clear_button_->setVisible(false);
  search_clear_button_->setFixedSize(18, 18);
  search_clear_button_->setStyleSheet(
      QStringLiteral(
          "QToolButton {"
          "  color: palette(mid);"
          "  background: transparent;"
          "  border: none;"
          "  font-size: 14px;"
          "  padding: 0px;"
          "}"
          "QToolButton:hover { color: palette(text); }"));
  toolbar_layout->addWidget(search_clear_button_);

  status_label_ = new QLabel(toolbar);
  status_label_->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 10px;"));
  status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  toolbar_layout->addWidget(status_label_);
  root->addWidget(toolbar);

  view_ = new LogViewWidget(this);
  root->addWidget(view_, 1);

  follow_timer_ = new QTimer(this);
  follow_timer_->setInterval(250);
  connect(follow_timer_, &QTimer::timeout, this, &LogPanel::onFollowTick);

  connect(settings_widget_, &LogSettingsWidget::configChanged, this, [this]() {
    const QString previous_topic = config_.topic;
    config_ = settings_widget_->config();
    applyConfigToUi();
    updateNamespaceUi();
    if (config_.topic != previous_topic) {
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
  updateStatusBar(view_->visibleEntryCount(), view_->totalEntryCount(),
                  view_->pinnedToBottom());
  follow_timer_->start();
}

LogPanel::~LogPanel() { unsubscribeTopic(); }

void LogPanel::installTitleBarTools(PanelDockWidget* dock) {
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
  settings_button_->setChecked(config_.settings_visible);
  layout->addWidget(settings_button_);
  connect(settings_button_, &QToolButton::toggled, this, &LogPanel::onToggleSettings);

  auto* more_button = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_more.svg")),
      tr("More"));
  more_button->setPopupMode(QToolButton::InstantPopup);
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
  more_button->setMenu(CreatePanelContextMenu(more_button, callbacks));
  layout->addWidget(more_button);

  dock->setTitleBarTools(tools);
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
  if (!config_.capture_glog) {
    return;
  }
  if (entry.source != QLatin1String("glog")) {
    return;
  }
  ingestEntry(entry);
}

void LogPanel::onChannelPayload(const std::string& payload) {
  const std::string channel = config_.topic.toStdString();
  std::string message_type;
  if (manager_ != nullptr) {
    for (const integration::ChannelInfo& info : manager_->channels()) {
      if (info.channel_name == channel) {
        message_type = info.message_type;
        break;
      }
    }
  }
  LogEntry entry =
      logEntryFromPayload(message_type, payload, QString::fromStdString(channel));
  if (entry.message.isEmpty()) {
    return;
  }
  ingestEntry(entry);
}

void LogPanel::resubscribeTopic() {
  unsubscribeTopic();
  const std::string channel = config_.topic.toStdString();
  if (channel.empty()) {
    return;
  }
  topic_subscription_id_ = integration::ChannelReaderRegistry::instance().subscribe(
      channel, [this](const std::string& payload) { onChannelPayload(payload); });
}

void LogPanel::unsubscribeTopic() {
  if (topic_subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(topic_subscription_id_);
    topic_subscription_id_ = 0;
  }
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

QWidget* LogPanel::settingsWidgetForInspector() { return settings_scroll_; }

void LogPanel::recallSettingsWidget() {
  if (settings_scroll_ == nullptr || settings_container_ == nullptr) {
    return;
  }
  if (settings_scroll_->parentWidget() == settings_container_) {
    return;
  }
  settings_scroll_->setParent(settings_container_);
  if (QLayout* layout = settings_container_->layout()) {
    layout->addWidget(settings_scroll_);
  }
  settings_scroll_->hide();
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

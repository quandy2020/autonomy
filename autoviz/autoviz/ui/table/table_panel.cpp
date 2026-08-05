/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/table/table_panel.hpp"

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
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/table/table_field_extractor.hpp"
#include "autoviz/ui/table/table_settings_widget.hpp"
#include "autoviz/ui/table/table_view_widget.hpp"

namespace autoviz {
namespace table {
namespace {

QToolButton* AddTitleToolButton(QWidget* parent, const QIcon& icon,
                                const QString& tip, bool checkable = false) {
  return CreatePlotTitleToolButton(parent, icon, tip, checkable);
}

}  // namespace

TablePanel::TablePanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultTablePanelConfig()), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(QStringLiteral("TablePanel { background: palette(window); }"));

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
  settings_widget_ = new TableSettingsWidget(manager_, settings_scroll_);
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

  view_ = new TableViewWidget(this);
  root->addWidget(view_, 1);

  connect(settings_widget_, &TableSettingsWidget::configChanged, this, [this]() {
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

TablePanel::~TablePanel() { unsubscribeChannel(); }

void TablePanel::installTitleBarTools(PanelDockWidget* dock) {
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
  connect(settings_button_, &QToolButton::toggled, this, &TablePanel::onToggleSettings);

  auto* more_button = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_more.svg")),
      tr("More"));
  more_button->setPopupMode(QToolButton::InstantPopup);
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("TableDock");
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

TablePanelConfig TablePanel::config() const { return config_; }

void TablePanel::setConfig(const TablePanelConfig& config) {
  config_ = config;
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  resubscribeChannel();
}

void TablePanel::cloneConfigFrom(const TablePanelConfig& config) {
  setConfig(config);
}

void TablePanel::setSettingsVisible(bool visible) {
  if (settings_container_ != nullptr) {
    settings_container_->setVisible(visible);
  }
  syncSettingsToolState();
}

bool TablePanel::settingsVisible() const {
  return settings_container_ != nullptr && settings_container_->isVisible();
}

void TablePanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ != nullptr) {
    settings_button_->blockSignals(true);
    settings_button_->setChecked(checked);
    settings_button_->blockSignals(false);
  }
}

void TablePanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ != nullptr) {
    expand_button_->blockSignals(true);
    expand_button_->setChecked(checked);
    expand_button_->blockSignals(false);
  }
}

QWidget* TablePanel::settingsWidgetForInspector() {
  if (settings_widget_ == nullptr) {
    return nullptr;
  }
  settings_widget_->setParent(nullptr);
  return settings_widget_;
}

void TablePanel::recallSettingsWidget() {
  if (settings_widget_ == nullptr || settings_scroll_ == nullptr) {
    return;
  }
  settings_widget_->setParent(settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
}

void TablePanel::refreshSettingsChannels() {
  if (settings_widget_ != nullptr) {
    settings_widget_->refreshChannels();
  }
}

void TablePanel::handleTableDrop(const QString& channel, const QString& array_path) {
  if (channel.isEmpty()) {
    return;
  }
  config_.channel = channel;
  if (!array_path.isEmpty()) {
    config_.array_path = array_path;
  }
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  resubscribeChannel();
  emit configChanged();
  emit activated();
}

void TablePanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

bool TablePanel::readDropPayload(const QMimeData* mime, QString* channel,
                                 QString* array_path) const {
  if (mime == nullptr || channel == nullptr || array_path == nullptr) {
    return false;
  }
  plot::PlotSeriesDragPayload payload;
  if (!plot::ReadPlotSeriesDragPayload(mime, &payload) || payload.channel.isEmpty()) {
    return false;
  }
  *channel = payload.channel;
  *array_path = payload.field_path;
  return true;
}

void TablePanel::dragEnterEvent(QDragEnterEvent* event) {
  if (event == nullptr) {
    return;
  }
  QString channel;
  QString array_path;
  if (readDropPayload(event->mimeData(), &channel, &array_path)) {
    event->acceptProposedAction();
    emit activated();
  }
}

void TablePanel::dragMoveEvent(QDragMoveEvent* event) {
  if (event == nullptr) {
    return;
  }
  QString channel;
  QString array_path;
  if (readDropPayload(event->mimeData(), &channel, &array_path)) {
    event->acceptProposedAction();
  }
}

void TablePanel::dropEvent(QDropEvent* event) {
  if (event == nullptr) {
    return;
  }
  QString channel;
  QString array_path;
  if (!readDropPayload(event->mimeData(), &channel, &array_path)) {
    return;
  }
  handleTableDrop(channel, array_path);
  event->acceptProposedAction();
}

void TablePanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

QString TablePanel::messageTypeForChannel(const QString& channel) const {
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

void TablePanel::resubscribeChannel() {
  unsubscribeChannel();
  if (config_.channel.isEmpty()) {
    view_->clearData();
    updateStatusBar();
    return;
  }
  subscribed_message_type_ = messageTypeForChannel(config_.channel).toStdString();
  subscription_id_ = integration::ChannelReaderRegistry::instance().subscribe(
      config_.channel.toStdString(),
      [this](const std::string& payload) { onChannelPayload(payload); });
}

void TablePanel::unsubscribeChannel() {
  if (subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(subscription_id_);
    subscription_id_ = 0;
  }
  subscribed_message_type_.clear();
}

void TablePanel::onChannelPayload(const std::string& payload) {
  ingestPayload(payload);
}

void TablePanel::ingestPayload(const std::string& payload) {
  if (subscribed_message_type_.empty() || payload.empty()) {
    return;
  }
  const std::optional<TableData> data = TableFieldExtractor::instance().extract(
      subscribed_message_type_, payload, config_.array_path.toStdString());
  if (!data.has_value()) {
    last_row_count_ = 0;
    view_->setStatusText(tr("No array data at path \"%1\"")
                             .arg(config_.array_path.isEmpty() ? tr("(auto)")
                                                               : config_.array_path));
    view_->clearData();
    updateStatusBar();
    return;
  }
  last_row_count_ = static_cast<int>(data->rows.size());
  view_->setTableData(*data);
  updateStatusBar();
}

void TablePanel::applyConfigToUi() {
  updateStatusBar();
}

void TablePanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ != nullptr) {
    settings_widget_->setConfig(config_);
  }
}

void TablePanel::syncSettingsToolState() {
  setSettingsButtonChecked(settingsVisible());
}

void TablePanel::updateStatusBar() {
  if (status_label_ == nullptr) {
    return;
  }
  if (config_.channel.isEmpty()) {
    status_label_->setText(tr("Drop a topic or repeated field from Topics"));
    return;
  }
  QString text = config_.channel;
  if (!config_.array_path.isEmpty()) {
    text += QStringLiteral(" · ") + config_.array_path;
  }
  if (last_row_count_ > 0) {
    text += tr(" · %1 rows").arg(last_row_count_);
  }
  status_label_->setText(text);
}

}  // namespace table
}  // namespace autoviz

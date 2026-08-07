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
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/table/table_field_extractor.hpp"
#include "autoviz/ui/table/table_settings_widget.hpp"
#include "autoviz/ui/table/table_view_widget.hpp"
#include "autoviz/variables/variable_path_utils.hpp"

namespace autoviz {
namespace table {
namespace {


}  // namespace

TablePanel::TablePanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultTablePanelConfig()), QWidget(parent) {
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
  settings_widget_ = new TableSettingsWidget(manager_, settings_scroll_);
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
  return SettingsScrollForInspector(settings_scroll_);
}

void TablePanel::recallSettingsWidget() {
  RecallSettingsScrollToContainer(settings_scroll_, settings_container_);
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

void TablePanel::refreshFromVariables() {
  if (!last_payload_.empty()) {
    ingestPayload(last_payload_);
  }
}

void TablePanel::ingestPayload(const std::string& payload) {
  last_payload_ = payload;
  if (subscribed_message_type_.empty() || payload.empty()) {
    return;
  }
  const QString resolved_path =
      manager_ != nullptr
          ? plot::ResolveMessagePath(config_.array_path, &manager_->variableStore())
          : config_.array_path;
  const std::optional<TableData> data = TableFieldExtractor::instance().extract(
      subscribed_message_type_, payload, resolved_path.toStdString());
  if (!data.has_value()) {
    last_row_count_ = 0;
    view_->setStatusText(tr("No array data at path \"%1\"")
                             .arg(resolved_path.isEmpty() ? tr("(auto)")
                                                          : resolved_path));
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
    status_label_->setText(tr("Drop a channel or repeated field from Channels"));
    return;
  }
  QString text = config_.channel;
  if (!config_.array_path.isEmpty()) {
    const QString resolved =
        manager_ != nullptr
            ? plot::ResolveMessagePath(config_.array_path,
                                     &manager_->variableStore())
            : config_.array_path;
    text += QStringLiteral(" · ") + resolved;
  }
  if (last_row_count_ > 0) {
    text += tr(" · %1 rows").arg(last_row_count_);
  }
  status_label_->setText(text);
}

}  // namespace table
}  // namespace autoviz

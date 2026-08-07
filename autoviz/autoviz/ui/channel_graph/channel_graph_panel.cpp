/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/channel_graph/channel_graph_panel.hpp"

#include <utility>

#include <QCheckBox>
#include <QComboBox>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QShowEvent>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/topology_graph_builder.hpp"
#include "autoviz/ui/channel_graph/channel_graph_view.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_title_tools.hpp"

namespace autoviz {
namespace channel_graph {
namespace {

constexpr int kAutoRefreshMs = 2000;
constexpr int kFilterDebounceMs = 250;

}  // namespace

ChannelGraphPanelConfig DefaultChannelGraphPanelConfig() {
  ChannelGraphPanelConfig config;
  config.show_services = true;
  config.show_channels = true;
  config.auto_refresh = true;
  return config;
}

ChannelGraphPanel::ChannelGraphPanel(common::VisualizationManager* manager,
                                     QWidget* parent)
    : manager_(manager), config_(DefaultChannelGraphPanelConfig()), QWidget(parent) {
  Q_UNUSED(manager_);
  setFocusPolicy(Qt::StrongFocus);
  setupUi();
  applyConfigToUi();
  rebuildPrefixCombo();
  refreshGraph();
}

void ChannelGraphPanel::setupUi() {
  ApplyPanelShell(this);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* toolbar = new QFrame(this);
  ApplyPanelToolbarChrome(toolbar);
  auto* toolbar_layout = new QHBoxLayout(toolbar);
  ApplyPanelToolbarLayout(toolbar_layout);

  auto* zoom_fit_button = new QToolButton(toolbar);
  zoom_fit_button->setToolTip(tr("Zoom to fit"));
  zoom_fit_button->setIcon(
      IconLoader::panelTitleIcon(QStringLiteral("plot.reset_view")));
  toolbar_layout->addWidget(zoom_fit_button);

  auto* refresh_button = new QToolButton(toolbar);
  refresh_button->setToolTip(tr("Refresh graph"));
  refresh_button->setIcon(IconLoader::menuIcon(QStringLiteral("file.recent")));
  toolbar_layout->addWidget(refresh_button);

  show_channels_check_ = new QCheckBox(tr("Channels"), toolbar);
  show_channels_check_->setChecked(true);
  show_channels_check_->setToolTip(tr("Show channel vertices in the graph"));
  toolbar_layout->addWidget(show_channels_check_);

  show_services_check_ = new QCheckBox(tr("Services"), toolbar);
  show_services_check_->setChecked(true);
  show_services_check_->setToolTip(tr("Show service vertices in the graph"));
  toolbar_layout->addWidget(show_services_check_);

  toolbar_layout->addWidget(new QLabel(tr("Group"), toolbar));
  prefix_combo_ = new QComboBox(toolbar);
  prefix_combo_->setMinimumWidth(120);
  prefix_combo_->setToolTip(tr("Filter channels by namespace prefix"));
  toolbar_layout->addWidget(prefix_combo_);

  filter_edit_ = new QLineEdit(toolbar);
  filter_edit_->setPlaceholderText(tr("Filter nodes, channels, services…"));
  filter_edit_->setClearButtonEnabled(true);
  filter_edit_->setMinimumWidth(160);
  StyleFilterLineEdit(filter_edit_);
  toolbar_layout->addWidget(filter_edit_, 1);

  auto_refresh_check_ = new QCheckBox(tr("Auto"), toolbar);
  auto_refresh_check_->setChecked(true);
  auto_refresh_check_->setToolTip(tr("Automatically refresh topology"));
  toolbar_layout->addWidget(auto_refresh_check_);

  root->addWidget(toolbar);

  auto* legend = new QLabel(
      tr("● Node (blue circle)  ▬ Channel (purple capsule bus)  ⬡ Service (red hexagon). "
         "Data flow: Writers → Channel → Readers. Edge labels: pub / sub / relay."),
      this);
  legend->setWordWrap(true);
  legend->setContentsMargins(PanelChromeLayout::kToolbarMarginH, 4,
                             PanelChromeLayout::kToolbarMarginH, 4);
  StyleHintLabel(legend);
  root->addWidget(legend);

  graph_view_ = new ChannelGraphView(this);
  graph_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  root->addWidget(graph_view_, 1);

  auto* status_bar = new QFrame(this);
  ApplyPanelFooterChrome(status_bar);
  auto* status_layout = new QHBoxLayout(status_bar);
  status_layout->setContentsMargins(PanelChromeLayout::kFooterMarginH,
                                    PanelChromeLayout::kFooterMarginV,
                                    PanelChromeLayout::kFooterMarginH,
                                    PanelChromeLayout::kFooterMarginV);
  status_label_ = new QLabel(status_bar);
  StylePanelStatusLabel(status_label_);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  status_layout->addWidget(status_label_, 1);
  root->addWidget(status_bar);

  refresh_timer_ = new QTimer(this);
  refresh_timer_->setInterval(kAutoRefreshMs);
  filter_timer_ = new QTimer(this);
  filter_timer_->setSingleShot(true);
  filter_timer_->setInterval(kFilterDebounceMs);

  connect(zoom_fit_button, &QToolButton::clicked, this,
          &ChannelGraphPanel::onZoomFitClicked);
  connect(refresh_button, &QToolButton::clicked, this,
          &ChannelGraphPanel::onRefreshClicked);
  connect(show_services_check_, &QCheckBox::toggled, this,
          &ChannelGraphPanel::onShowServicesToggled);
  connect(show_channels_check_, &QCheckBox::toggled, this,
          &ChannelGraphPanel::onShowChannelsToggled);
  connect(auto_refresh_check_, &QCheckBox::toggled, this,
          &ChannelGraphPanel::onAutoRefreshToggled);
  connect(prefix_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &ChannelGraphPanel::onPrefixChanged);
  connect(filter_edit_, &QLineEdit::textChanged, this,
          &ChannelGraphPanel::onFilterChanged);
  connect(filter_timer_, &QTimer::timeout, this, &ChannelGraphPanel::refreshGraph);
  connect(refresh_timer_, &QTimer::timeout, this, &ChannelGraphPanel::refreshGraph);
  connect(graph_view_, &ChannelGraphView::graphRendered, this,
          &ChannelGraphPanel::onGraphRendered);
}

void ChannelGraphPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("ChannelGraphDock");
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.split = [this](Qt::Orientation orientation) {
    emit panelSplitRequested(orientation);
  };
  callbacks.expand = [this]() { emit panelExpandRequested(); };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };

  PanelTitleBarOptions options;
  options.on_expand = [this]() { emit panelExpandRequested(); };

  const PanelTitleBarTools tools =
      CreateRvizPanelTitleBarTools(dock, callbacks, options);
  expand_button_ = tools.expand_button;
  dock->setTitleBarTools(tools.widget);
}

void ChannelGraphPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ == nullptr) {
    return;
  }
  expand_button_->blockSignals(true);
  expand_button_->setChecked(checked);
  expand_button_->blockSignals(false);
}

ChannelGraphPanelConfig ChannelGraphPanel::config() const { return config_; }

void ChannelGraphPanel::setConfig(const ChannelGraphPanelConfig& config) {
  config_ = config;
  applyConfigToUi();
  refreshGraph();
}

void ChannelGraphPanel::cloneConfigFrom(const ChannelGraphPanelConfig& config) {
  setConfig(config);
}

void ChannelGraphPanel::refreshGraph() {
  integration::TopologyGraphBuildOptions options;
  options.show_services = config_.show_services;
  options.show_channels = config_.show_channels;
  options.filter_text = config_.filter.toStdString();
  options.prefix_filter = config_.prefix_filter.toStdString();

  const integration::TopologyGraph graph = integration::BuildTopologyGraph(options);
  const bool topology_unchanged =
      !graph.topology_hash.empty() &&
      QString::fromStdString(graph.topology_hash) == last_topology_hash_;
  if (!topology_unchanged) {
    last_topology_hash_ = QString::fromStdString(graph.topology_hash);
  }
  graph_view_->setGraph(graph, topology_unchanged);
  if (graph.vertices.empty()) {
    updateStatusText(tr("No matching topology"));
  }
}

void ChannelGraphPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void ChannelGraphPanel::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  if (config_.auto_refresh) {
    refresh_timer_->start();
  }
  refreshGraph();
}

void ChannelGraphPanel::hideEvent(QHideEvent* event) {
  refresh_timer_->stop();
  QWidget::hideEvent(event);
}

void ChannelGraphPanel::onFilterChanged(const QString& text) {
  config_.filter = text;
  emit configChanged();
  filter_timer_->start();
}

void ChannelGraphPanel::onPrefixChanged(int index) {
  if (index < 0) {
    return;
  }
  config_.prefix_filter = prefix_combo_->itemData(index).toString();
  emit configChanged();
  refreshGraph();
}

void ChannelGraphPanel::onShowServicesToggled(bool enabled) {
  config_.show_services = enabled;
  emit configChanged();
  refreshGraph();
}

void ChannelGraphPanel::onShowChannelsToggled(bool enabled) {
  config_.show_channels = enabled;
  emit configChanged();
  graph_view_->resetSavedPositions();
  refreshGraph();
}

void ChannelGraphPanel::onAutoRefreshToggled(bool enabled) {
  config_.auto_refresh = enabled;
  emit configChanged();
  if (enabled && isVisible()) {
    refresh_timer_->start();
  } else {
    refresh_timer_->stop();
  }
}

void ChannelGraphPanel::onRefreshClicked() {
  rebuildPrefixCombo();
  last_topology_hash_.clear();
  refreshGraph();
}

void ChannelGraphPanel::onZoomFitClicked() { graph_view_->zoomToFit(); }

void ChannelGraphPanel::onGraphRendered(int vertex_count, int edge_count) {
  updateStatusText(tr("%1 vertices, %2 connections").arg(vertex_count).arg(edge_count));
}

void ChannelGraphPanel::applyConfigToUi() {
  show_services_check_->setChecked(config_.show_services);
  show_channels_check_->setChecked(config_.show_channels);
  auto_refresh_check_->setChecked(config_.auto_refresh);
  filter_edit_->setText(config_.filter);
}

void ChannelGraphPanel::rebuildPrefixCombo() {
  const QString current = config_.prefix_filter;
  prefix_combo_->blockSignals(true);
  prefix_combo_->clear();
  prefix_combo_->addItem(tr("All"), QString());
  for (const std::string& prefix : integration::ListChannelPrefixGroups()) {
    const QString value = QString::fromStdString(prefix);
    prefix_combo_->addItem(value, value);
  }
  const int index = prefix_combo_->findData(current);
  prefix_combo_->setCurrentIndex(index >= 0 ? index : 0);
  prefix_combo_->blockSignals(false);
  if (index < 0) {
    config_.prefix_filter.clear();
  }
}

void ChannelGraphPanel::updateStatusText(const QString& text) {
  status_label_->setText(text);
}

}  // namespace channel_graph
}  // namespace autoviz

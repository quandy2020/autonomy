/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_panel.hpp"

#include <chrono>

#include <QAbstractButton>
#include <QButtonGroup>
#include <QDragEnterEvent>
#include <QHBoxLayout>
#include <QDropEvent>
#include <QFile>
#include <QFileDialog>
#include <QFocusEvent>
#include <QFrame>
#include <QScrollArea>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QTextStream>
#include <QTimer>
#include <QToolButton>
#include <QWheelEvent>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/playback_controller.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/plot/plot_chart_widget.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/plot/plot_field_extractor.hpp"
#include "autoviz/ui/plot/plot_field_path.hpp"
#include "autoviz/ui/plot/plot_legend_widget.hpp"
#include "autoviz/ui/plot/plot_path_utils.hpp"
#include "autoviz/ui/plot/plot_settings_widget.hpp"
#include "autoviz/ui/plot/plot_view_sync.hpp"
#include "autoviz/variables/variable_path_utils.hpp"

namespace autoviz {
namespace plot {
namespace {

constexpr int kMaxPointsPerSeries = 5000;
constexpr int kMinSettingsWidth = 180;
constexpr int kMinChartWidth = 160;
constexpr int kDefaultSettingsWidth = 300;
constexpr int kSplitterHandleWidth = 6;
const char* kSeriesColors[] = {"#4e98e2", "#e74c3c", "#2ecc71", "#f39c12",
                               "#9b59b6", "#1abc9c", "#e84393", "#34495e"};

PlotSeriesRuntime* RuntimeAt(
    const std::vector<std::unique_ptr<PlotSeriesRuntime>>& runtime_series,
    int index) {
  if (index < 0 || index >= static_cast<int>(runtime_series.size()) ||
      runtime_series[index] == nullptr) {
    return nullptr;
  }
  return runtime_series[index].get();
}

bool SameSeriesDataSource(const PlotSeriesConfig& a,
                          const PlotSeriesConfig& b) {
  return a.channel == b.channel && a.field_path == b.field_path &&
         a.x_field_path == b.x_field_path &&
         a.custom_timestamp_path == b.custom_timestamp_path &&
         a.timestamp_mode == b.timestamp_mode;
}

}  // namespace

PlotPanel::PlotPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);

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
  settings_scroll_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  settings_widget_ = new PlotSettingsWidget(manager_, settings_scroll_);
  settings_widget_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
  settings_scroll_->setWidget(settings_widget_);
  settings_layout->addWidget(settings_scroll_);

  chart_ = new PlotChartWidget(this);
  chart_->setMinimumWidth(kMinChartWidth);
  root->addWidget(chart_, 1);

  legend_widget_ = new PlotLegendWidget(chart_);
  legend_widget_->raise();

  connect(settings_widget_, &PlotSettingsWidget::configChanged, this, [this]() {
    if (settings_widget_ != nullptr) {
      applySettings(settings_widget_->config());
    }
  });
  connect(settings_widget_, &PlotSettingsWidget::addSeriesRequested, this,
          &PlotPanel::onAddSeriesRequested);
  connect(settings_widget_, &PlotSettingsWidget::removeSeriesRequested, this,
          &PlotPanel::onRemoveSeriesRequested);

  chart_->installEventFilter(this);
  installEventFilter(this);

  connect(chart_, &PlotChartWidget::geometryChanged, this,
          &PlotPanel::updateLegendGeometry);
  connect(chart_, &PlotChartWidget::addSeriesRequested, this, [this]() {
    if (legend_widget_ != nullptr && !legend_widget_->isVisible()) {
      setLegendVisible(true);
    }
    onAddSeriesRequested();
  });
  connect(chart_, &PlotChartWidget::seekRequested, this, [this](double timestamp_sec) {
    if (manager_ == nullptr) {
      return;
    }
    integration::PlaybackController& playback = manager_->playback();
    if (!playback.currentFile().empty()) {
      playback.seekTo(timestamp_sec);
    }
  });
  connect(chart_, &PlotChartWidget::zoomToolToggleRequested, this, [this]() {
    if (chart_ == nullptr) {
      return;
    }
    if (chart_->interactionMode() == PlotInteractionMode::kZoom) {
      setChartInteractionMode(PlotInteractionMode::kSelect);
    } else {
      setChartInteractionMode(PlotInteractionMode::kZoom);
    }
  });
  connect(chart_, &PlotChartWidget::viewRangeChanged, this,
          [this](double min_x, double max_x, double /*min_y*/, double /*max_y*/) {
            if (config_.sync_with_other_plots &&
                config_.x_axis_mode == PlotXAxisMode::kTimestamp) {
              PlotViewSync::instance().publishXRange(this, min_x, max_x);
            }
          });
  connect(chart_, &PlotChartWidget::hoverStateChanged, this,
          &PlotPanel::updateLegendValues);
  connect(chart_, &PlotChartWidget::inspectEscapeRequested, this, [this]() {
    setChartInteractionMode(PlotInteractionMode::kSelect);
  });

  tick_timer_ = new QTimer(this);
  connect(tick_timer_, &QTimer::timeout, this, &PlotPanel::onTick);
  tick_timer_->start(50);

  syncSettingsWidgetFromConfig();
  applyConfigToUi();
  PlotViewSync::instance().registerPanel(this);
}

PlotPanel::~PlotPanel() {
  PlotViewSync::instance().unregisterPanel(this);
  for (auto& runtime : runtime_series_) {
    if (runtime != nullptr) {
      unsubscribeSeries(*runtime);
    }
  }
}

void PlotPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

bool PlotPanel::eventFilter(QObject* watched, QEvent* event) {
  if (event->type() == QEvent::MouseButtonPress) {
    Q_UNUSED(watched);
    emit activated();
  }
  if (event->type() == QEvent::Wheel && chart_ != nullptr) {
    auto* wheel_event = static_cast<QWheelEvent*>(event);
    QWidget* source = qobject_cast<QWidget*>(watched);
    if (source != nullptr) {
      const QPointF chart_pos =
          chart_->mapFrom(source, wheel_event->position());
      if (chart_->rect().contains(chart_pos.toPoint())) {
        chart_->handleWheelZoomAt(chart_pos, wheel_event->angleDelta().y(),
                                  wheel_event->modifiers());
        return true;
      }
    }
  }
  return QWidget::eventFilter(watched, event);
}

void PlotPanel::wheelEvent(QWheelEvent* event) {
  if (chart_ != nullptr) {
    const QPointF chart_pos = chart_->mapFrom(this, event->position());
    if (chart_->rect().contains(chart_pos.toPoint())) {
      chart_->handleWheelZoomAt(chart_pos, event->angleDelta().y(),
                                event->modifiers());
      event->accept();
      return;
    }
  }
  QWidget::wheelEvent(event);
}

void PlotPanel::setChartInteractionMode(PlotInteractionMode mode) {
  if (chart_ != nullptr) {
    chart_->setInteractionMode(mode);
  }
  syncInteractionToolState();
}

void PlotPanel::syncInteractionToolState() {
  if (interaction_tool_group_ == nullptr || chart_ == nullptr) {
    return;
  }
  QToolButton* target = select_tool_button_;
  switch (chart_->interactionMode()) {
    case PlotInteractionMode::kPan:
      target = pan_tool_button_;
      break;
    case PlotInteractionMode::kZoom:
      target = zoom_tool_button_;
      break;
    case PlotInteractionMode::kInspect:
      target = nullptr;
      break;
    case PlotInteractionMode::kSelect:
    default:
      target = select_tool_button_;
      break;
  }
  interaction_tool_group_->blockSignals(true);
  if (target != nullptr) {
    target->setChecked(true);
  } else {
    for (QAbstractButton* button : interaction_tool_group_->buttons()) {
      button->setChecked(false);
    }
  }
  interaction_tool_group_->blockSignals(false);
}

void PlotPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }

  const QString current_object_name =
      dock->property("panelTypeId").toString().isEmpty()
          ? dock->objectName()
          : dock->property("panelTypeId").toString();

  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = current_object_name;
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.split = [this](Qt::Orientation orientation) {
    emit panelSplitRequested(orientation);
  };
  callbacks.expand = [this]() { emit panelExpandRequested(); };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };
  callbacks.download_plot_csv = [this]() { exportPlotDataAsCsv(); };

  PanelTitleBarOptions options;
  options.show_reset = true;
  options.on_reset = [this]() {
    if (chart_ != nullptr) {
      chart_->resetView();
    }
  };
  options.show_settings = true;
  options.settings_checked = config_.settings_visible;
  options.on_settings_toggled = [this](bool visible) { onToggleSettings(visible); };
  options.on_expand = [this]() { emit panelExpandRequested(); };

  const PanelTitleBarTools tools =
      CreateRvizPanelTitleBarTools(dock, callbacks, options);
  settings_button_ = tools.settings_button;
  expand_button_ = tools.expand_button;
  dock->setTitleBarTools(tools.widget);

  if (auto* layout = qobject_cast<QHBoxLayout*>(tools.widget->layout())) {
    interaction_tool_group_ = new QButtonGroup(tools.widget);
    interaction_tool_group_->setExclusive(true);

    select_tool_button_ = CreatePlotTitleToolButton(
        tools.widget, IconLoader::panelTitleIcon(QStringLiteral("plot.select")),
        tr("Select (drag to pan, click to seek)"), true);
    pan_tool_button_ = CreatePlotTitleToolButton(
        tools.widget, IconLoader::panelTitleIcon(QStringLiteral("plot.pan")),
        tr("Pan"), true);
    zoom_tool_button_ = CreatePlotTitleToolButton(
        tools.widget, IconLoader::panelTitleIcon(QStringLiteral("plot.zoom")),
        tr("Zoom (drag rectangle)"), true);

    interaction_tool_group_->addButton(select_tool_button_);
    interaction_tool_group_->addButton(pan_tool_button_);
    interaction_tool_group_->addButton(zoom_tool_button_);

    layout->insertWidget(0, select_tool_button_);
    layout->insertWidget(1, pan_tool_button_);
    layout->insertWidget(2, zoom_tool_button_);
    layout->insertWidget(3, CreateTitleSeparator(tools.widget));

    connect(select_tool_button_, &QToolButton::clicked, this, [this]() {
      setChartInteractionMode(PlotInteractionMode::kSelect);
    });
    connect(pan_tool_button_, &QToolButton::clicked, this, [this]() {
      setChartInteractionMode(PlotInteractionMode::kPan);
    });
    connect(zoom_tool_button_, &QToolButton::clicked, this, [this]() {
      setChartInteractionMode(PlotInteractionMode::kZoom);
    });
  }

  setChartInteractionMode(PlotInteractionMode::kSelect);
  setLegendVisible(false);
}

PlotPanelConfig PlotPanel::config() const { return config_; }

void PlotPanel::setConfig(const PlotPanelConfig& config) {
  config_ = config;
  resubscribeAll();
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
}

void PlotPanel::cloneConfigFrom(const PlotPanelConfig& config) {
  for (auto& runtime : runtime_series_) {
    if (runtime != nullptr) {
      unsubscribeSeries(*runtime);
    }
  }
  runtime_series_.clear();

  config_ = config;

  runtime_series_.reserve(config_.series.size());
  for (const PlotSeriesConfig& series : config_.series) {
    auto runtime = std::make_unique<PlotSeriesRuntime>();
    runtime->config = series;
    runtime_series_.push_back(std::move(runtime));
  }
  for (int i = 0; i < static_cast<int>(runtime_series_.size()); ++i) {
    subscribeSeries(i);
  }

  if (chart_ != nullptr) {
    chart_->resetView();
    chart_->clearInspection();
  }
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
}

void PlotPanel::applySettings(const PlotPanelConfig& config) {
  const bool x_axis_changed = config_.x_axis_mode != config.x_axis_mode;
  config_ = config;
  if (x_axis_changed) {
    for (auto& runtime : runtime_series_) {
      if (runtime != nullptr) {
        runtime->points.clear();
        runtime->index_counter = 0;
      }
    }
  }
  mergeRuntimeWithConfig();
  applyConfigToUi();
  emit configChanged();
}

void PlotPanel::addSeries() { onAddSeriesRequested(); }

void PlotPanel::addSeriesFromTopic(const QString& channel,
                                   const QString& field_path) {
  handleSeriesDrop(channel, field_path);
}

void PlotPanel::removeSeries(int index) { onRemoveSeriesRequested(index); }

void PlotPanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ == nullptr) {
    return;
  }
  settings_button_->blockSignals(true);
  settings_button_->setChecked(checked);
  settings_button_->blockSignals(false);
}

void PlotPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ == nullptr) {
    return;
  }
  expand_button_->blockSignals(true);
  expand_button_->setChecked(checked);
  expand_button_->blockSignals(false);
}

void PlotPanel::syncSettingsToolState() {
  setSettingsButtonChecked(config_.settings_visible);
}

void PlotPanel::applySyncedXRange(double min_x, double max_x) {
  if (!config_.sync_with_other_plots ||
      config_.x_axis_mode != PlotXAxisMode::kTimestamp || chart_ == nullptr) {
    return;
  }
  chart_->applySyncedXRange(min_x, max_x);
}

void PlotPanel::updateLegendValues() {
  if (legend_widget_ == nullptr || chart_ == nullptr) {
    return;
  }
  legend_widget_->setShowValues(config_.show_legend_values);
  if (!config_.show_legend_values) {
    legend_widget_->setValueRows({});
    return;
  }
  const double x =
      chart_->hoverActive()
          ? chart_->hoverX()
          : (manager_ != nullptr ? manager_->simTimeSec() : 0.0);
  legend_widget_->setValueRows(chart_->valueRowsAtX(x));
}

void PlotPanel::setLegendVisible(bool visible) {
  if (legend_widget_ != nullptr) {
    legend_widget_->setVisible(visible);
    if (visible) {
      updateLegendGeometry();
    }
  }
  syncLegendToolState();
}

void PlotPanel::updateLegendGeometry() {
  if (legend_widget_ == nullptr || chart_ == nullptr) {
    return;
  }
  legend_widget_->adjustSize();
  const QRect anchor = chart_->legendAnchorRect();
  const QSize legend_size = legend_widget_->preferredSize();
  legend_widget_->setGeometry(anchor.left(), anchor.top(), legend_size.width(),
                              legend_size.height());
  legend_widget_->raise();
}

void PlotPanel::syncLegendToolState() {}

void PlotPanel::setSettingsVisible(bool visible) {
  config_.settings_visible = visible;
  syncSettingsToolState();
  emit configChanged();
}

bool PlotPanel::settingsVisible() const { return config_.settings_visible; }

QWidget* PlotPanel::settingsWidgetForInspector() {
  return SettingsScrollForInspector(settings_scroll_);
}

void PlotPanel::recallSettingsWidget() {
  RecallSettingsScrollToContainer(settings_scroll_, settings_container_);
}

void PlotPanel::refreshSettingsChannels() {
  if (settings_widget_ != nullptr) {
    settings_widget_->refreshChannelLists();
  }
}

void PlotPanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ == nullptr) {
    return;
  }
  settings_widget_->blockSignals(true);
  settings_widget_->setConfig(config_);
  settings_widget_->blockSignals(false);
}

void PlotPanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

void PlotPanel::onAddSeriesRequested() {
  PlotSeriesConfig series;
  series.label = PlotSettingsWidget::defaultSeriesLabel(config_.series.size());
  series.color = nextSeriesColor();
  ++color_cursor_;
  config_.series.push_back(series);

  auto runtime = std::make_unique<PlotSeriesRuntime>();
  runtime->config = series;
  runtime_series_.push_back(std::move(runtime));
  subscribeSeries(static_cast<int>(runtime_series_.size()) - 1);

  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  emit configChanged();
}

void PlotPanel::onRemoveSeriesRequested(int index) {
  if (PlotSeriesRuntime* runtime = RuntimeAt(runtime_series_, index)) {
    unsubscribeSeries(*runtime);
  }
  if (index >= 0 && index < runtime_series_.size()) {
    runtime_series_.erase(runtime_series_.begin() + index);
  }
  if (index >= 0 && index < config_.series.size()) {
    config_.series.removeAt(index);
  }
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  emit configChanged();
}

QColor PlotPanel::nextSeriesColor() const {
  return QColor(QString::fromLatin1(
      kSeriesColors[color_cursor_ % (sizeof(kSeriesColors) / sizeof(kSeriesColors[0]))]));
}

void PlotPanel::applyConfigToUi() {
  std::vector<const PlotSeriesRuntime*> series_view;
  series_view.reserve(runtime_series_.size());
  for (const auto& runtime : runtime_series_) {
    series_view.push_back(runtime.get());
  }
  chart_->setSeries(series_view);
  chart_->setXWindowSec(config_.x_window_sec);
  chart_->setXAxisMode(config_.x_axis_mode);
  chart_->setLockAxisScales(config_.lock_axis_scales);
  if (manager_ != nullptr) {
    chart_->setReferenceTimeSec(manager_->simTimeSec());
  }
  if (legend_widget_ != nullptr) {
    legend_widget_->setSeries(series_view);
    updateLegendValues();
    if (legend_widget_->isVisible()) {
      updateLegendGeometry();
    }
  }
  syncSettingsToolState();
  chart_->update();
}

void PlotPanel::resubscribeAll() {
  for (auto& runtime : runtime_series_) {
    if (runtime != nullptr) {
      unsubscribeSeries(*runtime);
    }
  }
  runtime_series_.clear();
  runtime_series_.reserve(config_.series.size());
  for (const PlotSeriesConfig& series : config_.series) {
    auto runtime = std::make_unique<PlotSeriesRuntime>();
    runtime->config = series;
    runtime_series_.push_back(std::move(runtime));
  }
  for (int i = 0; i < static_cast<int>(runtime_series_.size()); ++i) {
    subscribeSeries(i);
  }
}

void PlotPanel::mergeRuntimeWithConfig() {
  while (runtime_series_.size() > config_.series.size()) {
    unsubscribeSeries(*runtime_series_.back());
    runtime_series_.pop_back();
  }

  for (int i = 0; i < config_.series.size(); ++i) {
    const PlotSeriesConfig& series_config = config_.series[i];
    if (i >= static_cast<int>(runtime_series_.size())) {
      auto runtime = std::make_unique<PlotSeriesRuntime>();
      runtime->config = series_config;
      runtime_series_.push_back(std::move(runtime));
      subscribeSeries(i);
      continue;
    }

    PlotSeriesRuntime& runtime = *runtime_series_[i];
    if (SameSeriesDataSource(runtime.config, series_config)) {
      runtime.config = series_config;
      if (series_config.channel.isEmpty()) {
        unsubscribeSeries(runtime);
      } else if (runtime.subscription_id == 0) {
        subscribeSeries(i);
      }
      continue;
    }

    unsubscribeSeries(runtime);
    auto replacement = std::make_unique<PlotSeriesRuntime>();
    replacement->config = series_config;
    runtime_series_[i] = std::move(replacement);
    subscribeSeries(i);
  }
}

void PlotPanel::unsubscribeSeries(PlotSeriesRuntime& runtime) {
  if (runtime.subscription_id != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(
        static_cast<integration::ChannelReaderRegistry::SubscriptionId>(
            runtime.subscription_id));
    runtime.subscription_id = 0;
  }
}

void PlotPanel::subscribeSeries(int index) {
  PlotSeriesRuntime* runtime = RuntimeAt(runtime_series_, index);
  if (runtime == nullptr) {
    return;
  }
  unsubscribeSeries(*runtime);
  const std::string channel = runtime->config.channel.toStdString();
  if (channel.empty()) {
    return;
  }
  runtime->subscription_id =
      integration::ChannelReaderRegistry::instance().subscribe(
          channel, [this, index](const std::string& payload) {
            PlotSeriesRuntime* series_runtime =
                RuntimeAt(runtime_series_, index);
            if (series_runtime != nullptr) {
              series_runtime->queue.push(payload);
            }
          });
}

std::string PlotPanel::messageTypeForChannel(const std::string& channel) const {
  return MessageTypeForChannel(manager_, QString::fromStdString(channel));
}

void PlotPanel::trimSeriesPoints(PlotSeriesRuntime& runtime, double latest_x) {
  while (runtime.points.size() > kMaxPointsPerSeries) {
    runtime.points.pop_front();
  }
  if (config_.x_axis_mode != PlotXAxisMode::kTimestamp) {
    return;
  }
  const double min_x = latest_x - config_.x_window_sec;
  while (!runtime.points.empty() && runtime.points.front().x < min_x) {
    runtime.points.pop_front();
  }
}

double PlotPanel::resolveTimestampSec(
    const PlotSeriesRuntime& runtime, const std::string& message_type,
    const std::string& payload, double receive_time, double sim_time) const {
  if (runtime.config.timestamp_mode == PlotTimestampMode::kReceiveTime) {
    return receive_time;
  }
  const double fallback =
      runtime.config.timestamp_mode == PlotTimestampMode::kLogTime ? sim_time
                                                                   : receive_time;
  if (runtime.config.timestamp_mode == PlotTimestampMode::kCustomField) {
    const QString resolved_path =
        manager_ != nullptr
            ? ResolvePlotFieldPath(runtime.config.custom_timestamp_path,
                                   &manager_->variableStore())
            : runtime.config.custom_timestamp_path;
    const std::optional<double> custom =
        PlotFieldExtractor::instance().extractTimestamp(
            message_type, payload, resolved_path.toStdString(), fallback);
    return custom.value_or(fallback);
  }
  const std::optional<double> header_time =
      PlotFieldExtractor::instance().extractTimestamp(message_type, payload, "",
                                                      fallback);
  return header_time.value_or(fallback);
}

void PlotPanel::handleSeriesDrop(const QString& channel,
                                 const QString& field_path) {
  if (channel.isEmpty()) {
    return;
  }
  for (const PlotSeriesConfig& existing : config_.series) {
    if (existing.channel == channel && existing.field_path == field_path) {
      emit activated();
      return;
    }
  }
  PlotSeriesConfig series;
  series.channel = channel;
  series.field_path = field_path;
  if (field_path.isEmpty()) {
    series.label = channel.section(QLatin1Char('/'), -1);
  } else {
    series.label = field_path.section(QLatin1Char('.'), -1);
  }
  series.color = nextSeriesColor();
  ++color_cursor_;
  config_.series.push_back(series);

  auto runtime = std::make_unique<PlotSeriesRuntime>();
  runtime->config = series;
  runtime_series_.push_back(std::move(runtime));
  subscribeSeries(static_cast<int>(runtime_series_.size()) - 1);

  if (legend_widget_ != nullptr && !legend_widget_->isVisible()) {
    setLegendVisible(true);
  }
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  emit configChanged();
  emit activated();
}

void PlotPanel::dragEnterEvent(QDragEnterEvent* event) {
  if (event == nullptr) {
    return;
  }
  if (!ReadPlotSeriesDragPayloads(event->mimeData()).isEmpty()) {
    event->acceptProposedAction();
    emit activated();
  }
}

void PlotPanel::dragMoveEvent(QDragMoveEvent* event) {
  if (event == nullptr) {
    return;
  }
  if (!ReadPlotSeriesDragPayloads(event->mimeData()).isEmpty()) {
    event->acceptProposedAction();
  }
}

void PlotPanel::dropEvent(QDropEvent* event) {
  if (event == nullptr) {
    return;
  }
  const QVector<PlotSeriesDragPayload> payloads =
      ReadPlotSeriesDragPayloads(event->mimeData());
  if (payloads.isEmpty()) {
    return;
  }
  for (const PlotSeriesDragPayload& payload : payloads) {
    handleSeriesDrop(payload.channel, payload.field_path);
  }
  event->acceptProposedAction();
}

void PlotPanel::onTick() {
  if (manager_ == nullptr) {
    return;
  }
  const double receive_time =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const double sim_time = manager_->simTimeSec();
  chart_->setReferenceTimeSec(sim_time);

  for (auto& runtime_ptr : runtime_series_) {
    if (runtime_ptr == nullptr) {
      continue;
    }
    PlotSeriesRuntime& runtime = *runtime_ptr;
    if (!runtime.config.enabled || runtime.config.channel.isEmpty() ||
        runtime.config.field_path.isEmpty()) {
      continue;
    }
    const std::string channel = runtime.config.channel.toStdString();
    const std::string message_type = messageTypeForChannel(channel);
    const QString resolved_y_path =
        ResolvePlotFieldPath(runtime.config.field_path, &manager_->variableStore());
    const QString resolved_x_path = ResolvePlotFieldPath(
        runtime.config.x_field_path, &manager_->variableStore());
    const ParsedFieldPath y_path = ParseFieldPath(resolved_y_path);
    const ParsedFieldPath x_path = ParseFieldPath(resolved_x_path);
    while (auto payload = runtime.queue.pop()) {
      const double timestamp_sec = resolveTimestampSec(
          runtime, message_type, *payload, receive_time, sim_time);
      const std::optional<double> raw_y =
          PlotFieldExtractor::instance().extractNumeric(
              message_type, *payload, y_path.base_path.toStdString());
      if (!raw_y.has_value()) {
        continue;
      }
      const double y = ApplyPlotModifiers(
          *raw_y, timestamp_sec, y_path.modifiers, &runtime.last_raw_value,
          &runtime.last_timestamp_sec, &runtime.has_last_sample);

      PlotPoint point;
      if (config_.x_axis_mode == PlotXAxisMode::kIndex) {
        point.x = runtime.index_counter++;
        point.y = y;
      } else if (config_.x_axis_mode == PlotXAxisMode::kMessagePath) {
        if (x_path.base_path.isEmpty()) {
          continue;
        }
        const std::optional<double> raw_x =
            PlotFieldExtractor::instance().extractNumeric(
                message_type, *payload, x_path.base_path.toStdString());
        if (!raw_x.has_value()) {
          continue;
        }
        point.x = ApplyPlotModifiers(*raw_x, timestamp_sec, x_path.modifiers,
                                     nullptr, nullptr, nullptr);
        point.y = y;
        if (config_.message_path_mode == PlotMessagePathMode::kCurrent) {
          runtime.points.clear();
        }
      } else {
        point.x = timestamp_sec;
        point.y = y;
      }
      runtime.points.push_back(point);
      trimSeriesPoints(runtime, point.x);
    }
  }
  chart_->update();
  updateLegendValues();
}

void PlotPanel::exportPlotDataAsCsv() {
  const QString default_name =
      config_.title.isEmpty() ? tr("plot_data.csv")
                              : config_.title + QStringLiteral("_data.csv");
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Download plot data as CSV"), default_name,
      tr("CSV files (*.csv)"));
  if (path.isEmpty()) {
    return;
  }

  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::warning(this, tr("Export failed"),
                         tr("Could not write to %1").arg(path));
    return;
  }

  QTextStream out(&file);
  out << "x,y,label,channel,field_path\n";
  for (const auto& runtime_ptr : runtime_series_) {
    if (runtime_ptr == nullptr || !runtime_ptr->config.enabled) {
      continue;
    }
    const PlotSeriesConfig& series = runtime_ptr->config;
    const QString label =
        series.label.isEmpty() ? series.field_path : series.label;
    for (const PlotPoint& point : runtime_ptr->points) {
      out << QString::number(point.x, 'g', 12) << ','
          << QString::number(point.y, 'g', 12) << ','
          << label << ',' << series.channel << ','
          << series.field_path << '\n';
    }
  }
  file.close();
}

void PlotPanel::invalidateSeriesData() {
  for (auto& runtime_ptr : runtime_series_) {
    if (runtime_ptr == nullptr) {
      continue;
    }
    runtime_ptr->points.clear();
    runtime_ptr->index_counter = 0;
    runtime_ptr->has_last_sample = false;
  }
  chart_->update();
  updateLegendValues();
}

}  // namespace plot
}  // namespace autoviz

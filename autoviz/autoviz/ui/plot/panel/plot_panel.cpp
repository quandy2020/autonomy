/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/panel/plot_panel.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include <QAbstractButton>
#include <QApplication>
#include <QButtonGroup>
#include <QDragEnterEvent>
#include <QHBoxLayout>
#include <QDropEvent>
#include <QFile>
#include <QFileDialog>
#include <QIODevice>
#include <QFocusEvent>
#include <QFrame>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPixmap>
#include <QStyle>
#include <QTextStream>
#include <QTimer>
#include <QToolButton>
#include <QWheelEvent>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/common/interaction/channel_reader_registry.hpp"
#include "autoviz/common/interaction/playback_controller.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/panel/title_tools.hpp"
#include "autoviz/ui/plot/chart/plot_chart_widget.hpp"
#include "autoviz/ui/plot/core/plot_series_pipeline.hpp"
#include "autoviz/ui/plot/ui/plot_drag_mime.hpp"
#include "autoviz/ui/plot/ui/plot_legend_widget.hpp"
#include "autoviz/ui/plot/field/plot_path_utils.hpp"
#include "autoviz/ui/plot/ui/plot_settings_widget.hpp"
#include "autoviz/ui/plot/sync/plot_view_sync.hpp"
#include "autoviz/ui/plot/io/plot_xml_io.hpp"

namespace autoviz {
namespace plot {
namespace {

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
         a.x_field_path == b.x_field_path && a.x_channel == b.x_channel &&
         a.custom_timestamp_path == b.custom_timestamp_path &&
         a.channel_pattern == b.channel_pattern &&
         a.timestamp_mode == b.timestamp_mode &&
         a.binary_op == b.binary_op &&
         a.secondary_channel == b.secondary_channel &&
         a.secondary_field_path == b.secondary_field_path;
}

bool ChannelMatchesPattern(const QString& channel, const QString& pattern) {
  const QString trimmed = pattern.trimmed();
  if (trimmed.isEmpty()) {
    return false;
  }
  const QRegularExpression re(
      QRegularExpression::wildcardToRegularExpression(trimmed),
      QRegularExpression::CaseInsensitiveOption);
  return re.match(channel).hasMatch();
}

}  // namespace

PlotPanel::PlotPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), QWidget(parent) {
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
  connect(legend_widget_, &PlotLegendWidget::seriesVisibilityToggled, this,
          [this](int index) {
            if (index < 0 || index >= config_.series.size()) {
              return;
            }
            config_.series[index].enabled = !config_.series[index].enabled;
            if (index < static_cast<int>(runtime_series_.size()) &&
                runtime_series_[static_cast<std::size_t>(index)] != nullptr) {
              runtime_series_[static_cast<std::size_t>(index)]->config.enabled =
                  config_.series[index].enabled;
            }
            applyConfigToUi();
            syncSettingsWidgetFromConfig();
            emit configChanged();
          });

  connect(settings_widget_, &PlotSettingsWidget::configChanged, this, [this]() {
    if (settings_widget_ != nullptr) {
      applySettings(settings_widget_->config());
    }
  });
  connect(settings_widget_, &PlotSettingsWidget::addSeriesRequested, this,
          &PlotPanel::onAddSeriesRequested);
  connect(settings_widget_, &PlotSettingsWidget::removeSeriesRequested, this,
          &PlotPanel::onRemoveSeriesRequested);
  connect(settings_widget_, &PlotSettingsWidget::importXmlRequested, this,
          &PlotPanel::importPlotXml);
  connect(settings_widget_, &PlotSettingsWidget::exportXmlRequested, this,
          &PlotPanel::exportPlotXml);

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
          [this](double min_x, double max_x, double min_y, double max_y) {
            const QString group = effectiveSyncGroup();
            if (group.isEmpty()) {
              return;
            }
            if (config_.x_axis_mode == PlotXAxisMode::kTimestamp) {
              PlotViewSync::instance().publishXRange(this, group, min_x, max_x);
            }
            if (config_.sync_y_axis) {
              PlotViewSync::instance().publishYRange(this, group, min_y, max_y);
            }
          });
  connect(chart_, &PlotChartWidget::hoverStateChanged, this, [this]() {
    updateLegendValues();
    const QString group = effectiveSyncGroup();
    if (group.isEmpty() || !config_.sync_cursor || chart_ == nullptr) {
      return;
    }
    // Prefer latched scrub cursor; fall back to live hover.
    const bool active = chart_->latchActive() || chart_->hoverActive();
    const double x =
        chart_->latchActive() ? chart_->latchX() : chart_->hoverX();
    PlotViewSync::instance().publishCursorX(this, group, active, x);
  });
  connect(chart_, &PlotChartWidget::latchCursorChanged, this, [this]() {
    updateLegendValues();
    const QString group = effectiveSyncGroup();
    if (group.isEmpty() || !config_.sync_cursor || chart_ == nullptr) {
      return;
    }
    PlotViewSync::instance().publishCursorX(this, group, chart_->latchActive(),
                                            chart_->latchX());
  });
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
      target = inspect_tool_button_;
      break;
    case PlotInteractionMode::kBrush:
      target = brush_tool_button_;
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
  callbacks.download_plot_png = [this]() { exportPlotAsPng(); };

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
      CreatePanelTitleBarTools(dock, callbacks, options);
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
    inspect_tool_button_ = CreatePlotTitleToolButton(
        tools.widget, IconLoader::panelTitleIcon(QStringLiteral("plot.inspect")),
        tr("Inspect (click A/B points, Esc to clear)"), true);
    brush_tool_button_ = CreatePlotTitleToolButton(
        tools.widget, IconLoader::panelTitleIcon(QStringLiteral("plot.brush")),
        tr("Brush (drag X-range for min/max/mean/std/RMS)"), true);

    interaction_tool_group_->addButton(select_tool_button_);
    interaction_tool_group_->addButton(pan_tool_button_);
    interaction_tool_group_->addButton(zoom_tool_button_);
    interaction_tool_group_->addButton(inspect_tool_button_);
    interaction_tool_group_->addButton(brush_tool_button_);

    layout->insertWidget(0, select_tool_button_);
    layout->insertWidget(1, pan_tool_button_);
    layout->insertWidget(2, zoom_tool_button_);
    layout->insertWidget(3, inspect_tool_button_);
    layout->insertWidget(4, brush_tool_button_);
    layout->insertWidget(5, CreateTitleSeparator(tools.widget));

    legend_tool_button_ = CreatePlotTitleToolButton(
        tools.widget, IconLoader::panelTitleIcon(QStringLiteral("plot.legend")),
        tr("Show legend"), true);
    legend_tool_button_->setChecked(legend_widget_ != nullptr &&
                                    legend_widget_->isVisible());
    layout->insertWidget(6, legend_tool_button_);
    connect(legend_tool_button_, &QToolButton::toggled, this,
            [this](bool checked) { setLegendVisible(checked); });

    layout->insertWidget(7, CreateTitleSeparator(tools.widget));
    pause_tool_button_ = CreatePlotTitleToolButton(
        tools.widget, QApplication::style()->standardIcon(QStyle::SP_MediaPause),
        tr("Pause sampling"), true);
    clear_tool_button_ = CreatePlotTitleToolButton(
        tools.widget,
        QApplication::style()->standardIcon(QStyle::SP_DialogResetButton),
        tr("Clear plot data"), false);
    layout->insertWidget(8, pause_tool_button_);
    layout->insertWidget(9, clear_tool_button_);
    connect(pause_tool_button_, &QToolButton::toggled, this,
            [this](bool checked) { setPaused(checked); });
    connect(clear_tool_button_, &QToolButton::clicked, this,
            [this]() { clearPlotData(); });

    connect(select_tool_button_, &QToolButton::clicked, this, [this]() {
      setChartInteractionMode(PlotInteractionMode::kSelect);
    });
    connect(pan_tool_button_, &QToolButton::clicked, this, [this]() {
      setChartInteractionMode(PlotInteractionMode::kPan);
    });
    connect(zoom_tool_button_, &QToolButton::clicked, this, [this]() {
      setChartInteractionMode(PlotInteractionMode::kZoom);
    });
    connect(inspect_tool_button_, &QToolButton::clicked, this, [this]() {
      setChartInteractionMode(PlotInteractionMode::kInspect);
    });
    connect(brush_tool_button_, &QToolButton::clicked, this, [this]() {
      setChartInteractionMode(PlotInteractionMode::kBrush);
    });
  }

  setChartInteractionMode(PlotInteractionMode::kSelect);
  setLegendVisible(false);
  syncPauseToolState();
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
  const bool xy_subscribe_changed =
      (config_.x_axis_mode == PlotXAxisMode::kXY) !=
      (config.x_axis_mode == PlotXAxisMode::kXY);
  config_ = config;
  if (x_axis_changed) {
    for (auto& runtime : runtime_series_) {
      if (runtime != nullptr) {
        runtime->points.clear();
        runtime->secondary_samples.clear();
        runtime->x_samples.clear();
        runtime->modifier_window.clear();
        runtime->integral_accum = 0.0;
        runtime->index_counter = 0;
        runtime->has_last_sample = false;
      }
    }
  }
  mergeRuntimeWithConfig();
  if (xy_subscribe_changed) {
    for (int i = 0; i < static_cast<int>(runtime_series_.size()); ++i) {
      subscribeSeries(i);
    }
  }
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

QString PlotPanel::effectiveSyncGroup() const {
  // XY plots use spatial axes; do not join time-window sync groups.
  if (config_.x_axis_mode == PlotXAxisMode::kXY) {
    return {};
  }
  const QString group = config_.sync_group.trimmed();
  if (!group.isEmpty()) {
    return group;
  }
  return config_.sync_with_other_plots ? QStringLiteral("default") : QString();
}

void PlotPanel::applySyncedXRange(double min_x, double max_x) {
  if (effectiveSyncGroup().isEmpty() ||
      config_.x_axis_mode != PlotXAxisMode::kTimestamp || chart_ == nullptr) {
    return;
  }
  chart_->applySyncedXRange(min_x, max_x);
}

void PlotPanel::applySyncedYRange(double min_y, double max_y) {
  if (effectiveSyncGroup().isEmpty() || !config_.sync_y_axis ||
      chart_ == nullptr) {
    return;
  }
  chart_->applySyncedYRange(min_y, max_y);
  config_.y_auto_scale = false;
  config_.y_min = min_y;
  config_.y_max = max_y;
}

void PlotPanel::applySyncedCursorX(bool active, double x) {
  if (chart_ == nullptr) {
    return;
  }
  chart_->setSyncedCursorX(active, x);
}

void PlotPanel::setPaused(bool paused) {
  paused_ = paused;
  syncPauseToolState();
}

void PlotPanel::clearPlotData() { invalidateSeriesData(); }

void PlotPanel::syncPauseToolState() {
  if (pause_tool_button_ == nullptr) {
    return;
  }
  const QSignalBlocker block(pause_tool_button_);
  pause_tool_button_->setChecked(paused_);
  pause_tool_button_->setIcon(QApplication::style()->standardIcon(
      paused_ ? QStyle::SP_MediaPlay : QStyle::SP_MediaPause));
  pause_tool_button_->setToolTip(paused_ ? tr("Resume sampling")
                                         : tr("Pause sampling"));
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
  double x = manager_ != nullptr ? manager_->simTimeSec() : 0.0;
  if (chart_->latchActive()) {
    x = chart_->latchX();
  } else if (chart_->hoverActive()) {
    x = chart_->hoverX();
  }
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

void PlotPanel::syncLegendToolState() {
  if (legend_tool_button_ == nullptr) {
    return;
  }
  const bool visible =
      legend_widget_ != nullptr && legend_widget_->isVisible();
  legend_tool_button_->blockSignals(true);
  legend_tool_button_->setChecked(visible);
  legend_tool_button_->blockSignals(false);
}

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
  chart_->setYAutoScale(config_.y_auto_scale);
  chart_->setYRange(config_.y_min, config_.y_max);
  chart_->setYAutoScaleRight(config_.y_auto_scale_right);
  chart_->setYRangeRight(config_.y_min_right, config_.y_max_right);
  chart_->setShowGrid(config_.show_grid);
  chart_->setShowReferenceY(config_.show_reference_y);
  chart_->setReferenceY(config_.reference_y);
  chart_->setKeepRatioXy(config_.keep_ratio_xy);
  chart_->setShowReferenceTracker(config_.show_reference_tracker);
  chart_->setReferenceTrackerSec(config_.reference_tracker_sec);
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
  if (runtime.secondary_subscription_id != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(
        static_cast<integration::ChannelReaderRegistry::SubscriptionId>(
            runtime.secondary_subscription_id));
    runtime.secondary_subscription_id = 0;
  }
  if (runtime.x_subscription_id != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(
        static_cast<integration::ChannelReaderRegistry::SubscriptionId>(
            runtime.x_subscription_id));
    runtime.x_subscription_id = 0;
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

  if (UsesCrossChannelBinary(runtime->config)) {
    const std::string secondary =
        EffectiveSecondaryChannel(runtime->config).toStdString();
    if (!secondary.empty()) {
      runtime->secondary_subscription_id =
          integration::ChannelReaderRegistry::instance().subscribe(
              secondary, [this, index](const std::string& payload) {
                PlotSeriesRuntime* series_runtime =
                    RuntimeAt(runtime_series_, index);
                if (series_runtime != nullptr) {
                  series_runtime->secondary_queue.push(payload);
                }
              });
    }
  }

  if (UsesCrossChannelXy(runtime->config, config_.x_axis_mode)) {
    const std::string x_channel =
        EffectiveXChannel(runtime->config).toStdString();
    if (!x_channel.empty()) {
      runtime->x_subscription_id =
          integration::ChannelReaderRegistry::instance().subscribe(
              x_channel, [this, index](const std::string& payload) {
                PlotSeriesRuntime* series_runtime =
                    RuntimeAt(runtime_series_, index);
                if (series_runtime != nullptr) {
                  series_runtime->x_queue.push(payload);
                }
              });
    }
  }
}

std::string PlotPanel::messageTypeForChannel(const std::string& channel) const {
  return MessageTypeForChannel(manager_, QString::fromStdString(channel));
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
  series.label = CombinedPlotValuePath(channel, field_path);
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
  resolveChannelPatterns();
  const double receive_time =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const double sim_time = manager_->simTimeSec();
  chart_->setReferenceTimeSec(sim_time);

  SeriesPipelineContext ctx;
  ctx.x_axis_mode = config_.x_axis_mode;
  ctx.message_path_mode = config_.message_path_mode;
  ctx.x_window_sec = config_.x_window_sec;
  ctx.max_points_per_series = config_.max_points_per_series;
  ctx.receive_time = receive_time;
  ctx.sim_time = sim_time;
  ctx.message_type_for_channel = [this](const std::string& channel) {
    return messageTypeForChannel(channel);
  };

  bool dirty = false;
  for (auto& runtime_ptr : runtime_series_) {
    if (runtime_ptr == nullptr) {
      continue;
    }
    dirty = DrainAndSampleSeries(*runtime_ptr, ctx, paused_) || dirty;
  }
  const bool sim_moved = !std::isfinite(last_tick_sim_time_) ||
                         std::abs(sim_time - last_tick_sim_time_) > 1e-9;
  last_tick_sim_time_ = sim_time;
  if (dirty || sim_moved) {
    chart_->update();
    updateLegendValues();
  }
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
  out << "# autoviz plot export\n";
  out << "# title=" << config_.title << '\n';
  out << "# x_axis_mode=" << static_cast<int>(config_.x_axis_mode) << '\n';
  out << "# x_window_sec=" << config_.x_window_sec << '\n';
  for (const auto& runtime_ptr : runtime_series_) {
    if (runtime_ptr == nullptr || !runtime_ptr->config.enabled) {
      continue;
    }
    const PlotSeriesConfig& series = runtime_ptr->config;
    const QString label =
        SeriesLegendLabel(series.label, series.channel, series.field_path);
    out << "# series=" << label << '|' << series.channel << '|'
        << series.field_path << '|' << series.color.name() << '|'
        << (series.use_right_y ? "right" : "left") << '|'
        << series.line_size << '|'
        << (series.show_line ? "line" : "scatter") << '\n';
  }
  out << "x,y,label,channel,field_path,y_axis,color,line_size,show_line\n";
  for (const auto& runtime_ptr : runtime_series_) {
    if (runtime_ptr == nullptr || !runtime_ptr->config.enabled) {
      continue;
    }
    const PlotSeriesConfig& series = runtime_ptr->config;
    const QString label =
        SeriesLegendLabel(series.label, series.channel, series.field_path);
    const QString y_axis =
        series.use_right_y ? QStringLiteral("right") : QStringLiteral("left");
    for (const PlotPoint& point : runtime_ptr->points) {
      out << QString::number(point.x, 'g', 12) << ','
          << QString::number(point.y, 'g', 12) << ',' << label << ','
          << series.channel << ',' << series.field_path << ',' << y_axis << ','
          << series.color.name() << ',' << series.line_size << ','
          << (series.show_line ? 1 : 0) << '\n';
    }
  }
  file.close();
}

void PlotPanel::exportPlotAsPng() {
  if (chart_ == nullptr) {
    return;
  }
  const QString default_name =
      config_.title.isEmpty() ? tr("plot.png")
                              : config_.title + QStringLiteral(".png");
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Download plot as PNG"), default_name,
      tr("PNG images (*.png)"));
  if (path.isEmpty()) {
    return;
  }

  const QPixmap pixmap = chart_->grab();
  if (pixmap.isNull() || !pixmap.save(path, "PNG")) {
    QMessageBox::warning(this, tr("Export failed"),
                         tr("Could not write to %1").arg(path));
  }
}

void PlotPanel::invalidateSeriesData() {
  for (auto& runtime_ptr : runtime_series_) {
    if (runtime_ptr == nullptr) {
      continue;
    }
    runtime_ptr->points.clear();
    runtime_ptr->secondary_samples.clear();
    runtime_ptr->x_samples.clear();
    runtime_ptr->modifier_window.clear();
    runtime_ptr->integral_accum = 0.0;
    runtime_ptr->index_counter = 0;
    runtime_ptr->has_last_sample = false;
  }
  chart_->update();
  updateLegendValues();
}

void PlotPanel::resolveChannelPatterns() {
  if (manager_ == nullptr) {
    return;
  }
  bool changed = false;
  const auto& channels = manager_->channels();
  for (int i = 0; i < config_.series.size(); ++i) {
    PlotSeriesConfig& series = config_.series[i];
    const QString pattern = series.channel_pattern.trimmed();
    if (pattern.isEmpty()) {
      continue;
    }
    QString matched;
    for (const auto& info : channels) {
      const QString name = QString::fromStdString(info.channel_name);
      if (ChannelMatchesPattern(name, pattern)) {
        matched = name;
        break;
      }
    }
    if (matched.isEmpty() || matched == series.channel) {
      continue;
    }
    series.channel = matched;
    if (i < static_cast<int>(runtime_series_.size()) &&
        runtime_series_[i] != nullptr) {
      runtime_series_[i]->config.channel = matched;
    }
    changed = true;
  }
  if (!changed) {
    return;
  }
  mergeRuntimeWithConfig();
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  emit configChanged();
}

void PlotPanel::importPlotXml() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Import plot XML"), QString(),
      tr("Plot XML (*.xml);;All files (*)"));
  if (path.isEmpty()) {
    return;
  }
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    QMessageBox::warning(this, tr("Import failed"),
                         tr("Could not read %1").arg(path));
    return;
  }
  QString error;
  PlotPanelConfig imported;
  if (!ImportPlotXml(file.readAll(), &imported, &error)) {
    QMessageBox::warning(this, tr("Import failed"),
                         error.isEmpty() ? tr("Invalid plot XML") : error);
    return;
  }
  if (imported.title.trimmed().isEmpty()) {
    imported.title = config_.title;
  }
  setConfig(imported);
  emit configChanged();
}

void PlotPanel::exportPlotXml() {
  const QString default_name =
      config_.title.isEmpty() ? tr("plot.xml")
                              : config_.title + QStringLiteral(".xml");
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Export plot XML"), default_name,
      tr("Plot XML (*.xml);;All files (*)"));
  if (path.isEmpty()) {
    return;
  }
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    QMessageBox::warning(this, tr("Export failed"),
                         tr("Could not write %1").arg(path));
    return;
  }
  file.write(ExportPlotXml(config_));
}

}  // namespace plot
}  // namespace autoviz

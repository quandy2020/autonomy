/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/state_transitions/state_transition_panel.hpp"

#include <chrono>

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMimeData>
#include <QScrollArea>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/playback_controller.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/indicator/indicator_field_extractor.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"
#include "autoviz/ui/plot/plot_field_extractor.hpp"
#include "autoviz/ui/state_transitions/state_mapping.hpp"
#include "autoviz/ui/state_transitions/state_transition_settings_widget.hpp"
#include "autoviz/ui/state_transitions/state_transition_view_widget.hpp"
#include "autoviz/variables/variable_path_utils.hpp"

namespace autoviz {
namespace state_transitions {
namespace {

constexpr int kMaxSegmentsPerSeries = 2000;

QToolButton* AddTitleToolButton(QWidget* parent, const QIcon& icon,
                                const QString& tip, bool checkable = false) {
  return CreatePlotTitleToolButton(parent, icon, tip, checkable);
}

StateTransitionSeriesRuntime* RuntimeAt(
    const std::vector<std::unique_ptr<StateTransitionSeriesRuntime>>& runtime_series,
    int index) {
  if (index < 0 || index >= static_cast<int>(runtime_series.size()) ||
      runtime_series[index] == nullptr) {
    return nullptr;
  }
  return runtime_series[index].get();
}

}  // namespace

StateTransitionPanel::StateTransitionPanel(common::VisualizationManager* manager,
                                             QWidget* parent)
    : manager_(manager),
      config_(DefaultStateTransitionPanelConfig()),
      QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(
      QStringLiteral("StateTransitionPanel { background: palette(window); }"));

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
  settings_widget_ = new StateTransitionSettingsWidget(manager_, settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
  settings_layout->addWidget(settings_scroll_);

  view_ = new StateTransitionViewWidget(this);

  footer_ = new QFrame(this);
  footer_->setFixedHeight(18);
  footer_->setStyleSheet(PanelFooterStyle());
  auto* footer_layout = new QHBoxLayout(footer_);
  footer_layout->setContentsMargins(6, 0, 6, 0);
  footer_layout->setSpacing(0);
  status_label_ = new QLabel(footer_);
  status_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  status_label_->setStyleSheet(PanelStatusLabelStyle());
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  footer_layout->addWidget(status_label_, 0, Qt::AlignLeft);

  root->addWidget(view_, 1);
  root->addWidget(footer_);

  tick_timer_ = new QTimer(this);
  tick_timer_->setInterval(33);
  connect(tick_timer_, &QTimer::timeout, this, &StateTransitionPanel::onTick);

  connect(settings_widget_, &StateTransitionSettingsWidget::configChanged, this,
          [this]() {
            config_ = settings_widget_->config();
            applyConfigToUi();
            resubscribeAll();
            emit configChanged();
          });
  connect(settings_widget_, &StateTransitionSettingsWidget::addSeriesRequested, this,
          &StateTransitionPanel::onAddSeriesRequested);
  connect(settings_widget_, &StateTransitionSettingsWidget::removeSeriesRequested,
          this, &StateTransitionPanel::onRemoveSeriesRequested);
  connect(view_, &StateTransitionViewWidget::addSeriesRequested, this,
          &StateTransitionPanel::onAddSeriesRequested);
  connect(view_, &StateTransitionViewWidget::seekRequested, this,
          [this](double timestamp_sec) {
            if (manager_ == nullptr) {
              return;
            }
            integration::PlaybackController& playback = manager_->playback();
            if (!playback.currentFile().empty()) {
              playback.seekTo(timestamp_sec);
            }
          });
  connect(view_, &StateTransitionViewWidget::boxZoomModeChanged, this,
          [this](bool enabled) {
            if (box_zoom_button_ != nullptr) {
              box_zoom_button_->blockSignals(true);
              box_zoom_button_->setChecked(enabled);
              box_zoom_button_->blockSignals(false);
            }
          });

  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  updateStatusBar();
  tick_timer_->start();
}

StateTransitionPanel::~StateTransitionPanel() {
  for (auto& runtime : runtime_series_) {
    if (runtime != nullptr) {
      unsubscribeSeries(*runtime);
    }
  }
}

void StateTransitionPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  auto* tools = new QWidget(dock);
  tools->setStyleSheet(PlotTitleToolsStyleSheet());
  auto* layout = new QHBoxLayout(tools);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  auto* reset_view_button = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_reset_view.svg")),
      tr("Reset view"));
  layout->addWidget(reset_view_button);
  connect(reset_view_button, &QToolButton::clicked, this, [this]() {
    if (view_ != nullptr) {
      view_->resetView();
    }
  });

  box_zoom_button_ = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_zoom.svg")),
      tr("Box zoom (Z)"), true);
  layout->addWidget(box_zoom_button_);
  connect(box_zoom_button_, &QToolButton::toggled, this, [this](bool checked) {
    if (view_ != nullptr) {
      view_->setBoxZoomMode(checked);
    }
  });

  layout->addWidget(CreateTitleSeparator(tools));

  auto* split_right = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_split_right.svg")),
      tr("Split right"));
  layout->addWidget(split_right);
  connect(split_right, &QToolButton::clicked, this,
          [this]() { emit panelSplitRequested(Qt::Horizontal); });

  auto* split_down = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_split_down.svg")),
      tr("Split down"));
  layout->addWidget(split_down);
  connect(split_down, &QToolButton::clicked, this,
          [this]() { emit panelSplitRequested(Qt::Vertical); });

  layout->addWidget(CreateTitleSeparator(tools));

  settings_button_ = AddTitleToolButton(
      tools, IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_settings.svg")),
      tr("Settings"), true);
  layout->addWidget(settings_button_);
  connect(settings_button_, &QToolButton::toggled, this,
          &StateTransitionPanel::onToggleSettings);

  expand_button_ = AddTitleToolButton(
      tools, IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_fullscreen.svg")),
      tr("Expand"), true);
  layout->addWidget(expand_button_);
  connect(expand_button_, &QToolButton::clicked, this,
          [this]() { emit panelExpandRequested(); });

  auto* more_button = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_more.svg")),
      tr("More"));
  more_button->setPopupMode(QToolButton::InstantPopup);
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("StateTransitionDock");
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

StateTransitionPanelConfig StateTransitionPanel::config() const { return config_; }

void StateTransitionPanel::setConfig(const StateTransitionPanelConfig& config) {
  config_ = config;
  runtime_series_.clear();
  runtime_series_.reserve(config_.series.size());
  for (const StateTransitionSeriesConfig& series : config_.series) {
    auto runtime = std::make_unique<StateTransitionSeriesRuntime>();
    runtime->config = series;
    runtime_series_.push_back(std::move(runtime));
  }
  for (int i = 0; i < static_cast<int>(runtime_series_.size()); ++i) {
    subscribeSeries(i);
  }
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  setSettingsVisible(config_.settings_visible);
}

void StateTransitionPanel::cloneConfigFrom(
    const StateTransitionPanelConfig& config) {
  setConfig(config);
}

void StateTransitionPanel::setSettingsVisible(bool visible) {
  config_.settings_visible = visible;
  if (settings_container_ != nullptr) {
    settings_container_->setVisible(false);
  }
  setSettingsButtonChecked(visible);
}

bool StateTransitionPanel::settingsVisible() const {
  return config_.settings_visible;
}

void StateTransitionPanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ != nullptr) {
    settings_button_->blockSignals(true);
    settings_button_->setChecked(checked);
    settings_button_->blockSignals(false);
  }
}

void StateTransitionPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ != nullptr) {
    expand_button_->blockSignals(true);
    expand_button_->setChecked(checked);
    expand_button_->blockSignals(false);
  }
}

QWidget* StateTransitionPanel::settingsWidgetForInspector() {
  return SettingsScrollForInspector(settings_scroll_);
}

void StateTransitionPanel::recallSettingsWidget() {
  RecallSettingsScrollToContainer(settings_scroll_, settings_container_);
}

void StateTransitionPanel::refreshSettingsChannels() {
  if (settings_widget_ != nullptr) {
    settings_widget_->refreshChannels();
  }
}

void StateTransitionPanel::refreshFromVariables() { invalidateSeriesData(); }

void StateTransitionPanel::invalidateSeriesData() {
  for (auto& runtime : runtime_series_) {
    if (runtime == nullptr) {
      continue;
    }
    runtime->segments.clear();
    runtime->last_state_key.clear();
    runtime->has_last_state = false;
  }
  applyConfigToUi();
}

void StateTransitionPanel::handleSeriesDrop(const QString& channel,
                                            const QString& field_path) {
  if (channel.isEmpty() || field_path.isEmpty()) {
    return;
  }
  StateTransitionSeriesConfig series;
  series.channel = channel;
  series.field_path = field_path;
  series.label = field_path.section(QLatin1Char('.'), -1);
  config_.series.push_back(series);
  auto runtime = std::make_unique<StateTransitionSeriesRuntime>();
  runtime->config = series;
  runtime_series_.push_back(std::move(runtime));
  subscribeSeries(static_cast<int>(runtime_series_.size()) - 1);
  syncSettingsWidgetFromConfig();
  applyConfigToUi();
  emit configChanged();
  emit activated();
}

void StateTransitionPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void StateTransitionPanel::dragEnterEvent(QDragEnterEvent* event) {
  if (event == nullptr) {
    return;
  }
  plot::PlotSeriesDragPayload payload;
  if (plot::ReadPlotSeriesDragPayload(event->mimeData(), &payload) &&
      !payload.channel.isEmpty() && !payload.field_path.isEmpty()) {
    event->acceptProposedAction();
    emit activated();
  }
}

void StateTransitionPanel::dragMoveEvent(QDragMoveEvent* event) {
  if (event == nullptr) {
    return;
  }
  plot::PlotSeriesDragPayload payload;
  if (plot::ReadPlotSeriesDragPayload(event->mimeData(), &payload) &&
      !payload.channel.isEmpty() && !payload.field_path.isEmpty()) {
    event->acceptProposedAction();
  }
}

void StateTransitionPanel::dropEvent(QDropEvent* event) {
  if (event == nullptr) {
    return;
  }
  plot::PlotSeriesDragPayload payload;
  if (!plot::ReadPlotSeriesDragPayload(event->mimeData(), &payload) ||
      payload.channel.isEmpty() || payload.field_path.isEmpty()) {
    return;
  }
  handleSeriesDrop(payload.channel, payload.field_path);
  event->acceptProposedAction();
}

void StateTransitionPanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

void StateTransitionPanel::onAddSeriesRequested() {
  StateTransitionSeriesConfig series;
  series.label = tr("Series %1").arg(config_.series.size() + 1);
  config_.series.push_back(series);
  auto runtime = std::make_unique<StateTransitionSeriesRuntime>();
  runtime->config = series;
  runtime_series_.push_back(std::move(runtime));
  syncSettingsWidgetFromConfig();
  applyConfigToUi();
  emit configChanged();
}

void StateTransitionPanel::onRemoveSeriesRequested(int index) {
  if (StateTransitionSeriesRuntime* runtime = RuntimeAt(runtime_series_, index)) {
    unsubscribeSeries(*runtime);
  }
  if (index >= 0 && index < config_.series.size()) {
    config_.series.removeAt(index);
  }
  if (index >= 0 && index < static_cast<int>(runtime_series_.size())) {
    runtime_series_.erase(runtime_series_.begin() + index);
  }
  syncSettingsWidgetFromConfig();
  applyConfigToUi();
  emit configChanged();
}

void StateTransitionPanel::applyConfigToUi() {
  if (view_ == nullptr) {
    return;
  }
  view_->setPanelConfig(config_);
  std::vector<const StateTransitionSeriesRuntime*> series_view;
  series_view.reserve(runtime_series_.size());
  for (const auto& runtime : runtime_series_) {
    series_view.push_back(runtime.get());
  }
  view_->setSeries(series_view);
  updateStatusBar();
}

void StateTransitionPanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ != nullptr) {
    settings_widget_->setConfig(config_);
  }
}

void StateTransitionPanel::resubscribeAll() {
  for (int i = 0; i < static_cast<int>(runtime_series_.size()); ++i) {
    if (StateTransitionSeriesRuntime* runtime = RuntimeAt(runtime_series_, i)) {
      unsubscribeSeries(*runtime);
    }
    if (i < config_.series.size()) {
      runtime_series_[i]->config = config_.series[i];
    }
    subscribeSeries(i);
  }
}

void StateTransitionPanel::unsubscribeSeries(StateTransitionSeriesRuntime& runtime) {
  if (runtime.subscription_id != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(
        runtime.subscription_id);
    runtime.subscription_id = 0;
  }
}

void StateTransitionPanel::subscribeSeries(int index) {
  StateTransitionSeriesRuntime* runtime = RuntimeAt(runtime_series_, index);
  if (runtime == nullptr) {
    return;
  }
  if (runtime->config.channel.isEmpty() || !runtime->config.enabled) {
    unsubscribeSeries(*runtime);
    return;
  }
  unsubscribeSeries(*runtime);
  const std::string channel = runtime->config.channel.toStdString();
  runtime->subscription_id = integration::ChannelReaderRegistry::instance().subscribe(
      channel, [this, index](const std::string& payload) {
        ingestPayload(index, payload);
      });
}

std::string StateTransitionPanel::messageTypeForChannel(
    const std::string& channel) const {
  if (manager_ == nullptr) {
    return {};
  }
  for (const integration::ChannelInfo& info : manager_->channels()) {
    if (info.channel_name == channel) {
      return info.message_type;
    }
  }
  return {};
}

double StateTransitionPanel::resolveTimestampSec(
    const StateTransitionSeriesRuntime& runtime,
    const std::string& message_type, const std::string& payload,
    double receive_time, double sim_time) const {
  if (runtime.config.timestamp_mode == plot::PlotTimestampMode::kReceiveTime) {
    return receive_time;
  }
  const double fallback =
      runtime.config.timestamp_mode == plot::PlotTimestampMode::kLogTime
          ? sim_time
          : receive_time;
  if (runtime.config.timestamp_mode == plot::PlotTimestampMode::kCustomField) {
    const QString resolved_path =
        manager_ != nullptr
            ? plot::ResolveMessagePath(runtime.config.custom_timestamp_path,
                                       &manager_->variableStore())
            : runtime.config.custom_timestamp_path;
    const std::optional<double> custom =
        plot::PlotFieldExtractor::instance().extractTimestamp(
            message_type, payload, resolved_path.toStdString(), fallback);
    return custom.value_or(fallback);
  }
  return fallback;
}

void StateTransitionPanel::ingestPayload(int series_index,
                                         const std::string& payload) {
  StateTransitionSeriesRuntime* runtime = RuntimeAt(runtime_series_, series_index);
  if (runtime == nullptr || !runtime->config.enabled ||
      runtime->config.channel.isEmpty() || runtime->config.field_path.isEmpty()) {
    return;
  }
  runtime->queue.push(payload);
}

void StateTransitionPanel::trimSeriesSegments(
    StateTransitionSeriesRuntime& runtime, double latest_time) {
  while (runtime.segments.size() > kMaxSegmentsPerSeries) {
    runtime.segments.pop_front();
  }
  if (config_.x_axis_mode != StateXAxisMode::kSlidingWindow) {
    return;
  }
  const double min_time = latest_time - config_.x_window_sec * 2.0;
  while (!runtime.segments.empty() && runtime.segments.front().end_time < min_time) {
    runtime.segments.pop_front();
  }
}

void StateTransitionPanel::maybeInferMappingRule(
    int series_index, const indicator::IndicatorFieldValue& value) {
  if (series_index < 0 || series_index >= config_.series.size()) {
    return;
  }
  StateTransitionSeriesRuntime* runtime = RuntimeAt(runtime_series_, series_index);
  if (runtime == nullptr) {
    return;
  }
  if (!AppendInferredMappingRuleIfNeeded(runtime->config.mappings, value)) {
    return;
  }
  config_.series[series_index].mappings = runtime->config.mappings;
  syncSettingsWidgetFromConfig();
  emit configChanged();
}

void StateTransitionPanel::onTick() {
  if (manager_ == nullptr || view_ == nullptr) {
    return;
  }
  const double receive_time =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const double sim_time = manager_->simTimeSec();
  view_->setReferenceTimeSec(sim_time);

  for (int i = 0; i < static_cast<int>(runtime_series_.size()); ++i) {
    StateTransitionSeriesRuntime* runtime = RuntimeAt(runtime_series_, i);
    if (runtime == nullptr || !runtime->config.enabled ||
        runtime->config.channel.isEmpty() || runtime->config.field_path.isEmpty()) {
      continue;
    }
    const std::string channel = runtime->config.channel.toStdString();
    const std::string message_type = messageTypeForChannel(channel);
    const QString resolved_path =
        plot::ResolveMessagePath(runtime->config.field_path,
                                 &manager_->variableStore());

    while (auto payload = runtime->queue.pop()) {
      const double timestamp_sec = resolveTimestampSec(
          *runtime, message_type, *payload, receive_time, sim_time);
      const std::optional<indicator::IndicatorFieldValue> value =
          indicator::IndicatorFieldExtractor::instance().extract(
              message_type, *payload, resolved_path.toStdString());
      if (!value.has_value()) {
        continue;
      }
      maybeInferMappingRule(i, *value);
      const ResolvedStateDisplay display =
          ResolveStateDisplay(*value, runtime->config.mappings);
      if (runtime->has_last_state && runtime->last_state_key == display.state_key &&
          !runtime->segments.empty()) {
        runtime->segments.back().end_time = timestamp_sec;
      } else {
        StateSegment segment;
        segment.start_time = timestamp_sec;
        segment.end_time = timestamp_sec;
        segment.state_key = display.state_key;
        segment.label = display.label;
        segment.color = display.color;
        runtime->segments.push_back(segment);
        runtime->last_state_key = display.state_key;
        runtime->has_last_state = true;
      }
      trimSeriesSegments(*runtime, timestamp_sec);
    }

    if (!runtime->segments.empty()) {
      runtime->segments.back().end_time = sim_time;
    }
  }
  applyConfigToUi();
}

void StateTransitionPanel::updateStatusBar() {
  if (status_label_ == nullptr) {
    return;
  }
  int enabled_count = 0;
  for (const auto& runtime : runtime_series_) {
    if (runtime != nullptr && runtime->config.enabled) {
      ++enabled_count;
    }
  }
  if (enabled_count == 0) {
    status_label_->setText(
        tr("Drag discrete fields from Channels or click + to add a series"));
    return;
  }
  status_label_->setText(tr("%1 series").arg(enabled_count));
}

}  // namespace state_transitions
}  // namespace autoviz

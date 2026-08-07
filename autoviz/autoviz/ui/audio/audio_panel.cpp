/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/audio/audio_panel.hpp"

#include <vector>

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QMimeData>
#include <QScrollArea>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/playback_controller.hpp"
#include "autoviz/ui/audio/audio_playback_engine.hpp"
#include "autoviz/ui/audio/audio_settings_widget.hpp"
#include "autoviz/ui/audio/audio_waveform_widget.hpp"
#include "autoviz/ui/audio/raw_audio_parser.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/plot/plot_drag_mime.hpp"

namespace autoviz {
namespace audio_panel {
namespace {


bool ReadDropChannel(const QMimeData* mime, QString* channel) {
  if (mime == nullptr || channel == nullptr) {
    return false;
  }
  const QVector<plot::PlotSeriesDragPayload> payloads =
      plot::ReadPlotSeriesDragPayloads(mime);
  if (!payloads.isEmpty() && !payloads.front().channel.isEmpty()) {
    *channel = payloads.front().channel;
    return true;
  }
  plot::PlotSeriesDragPayload payload;
  if (plot::ReadPlotSeriesDragPayload(mime, &payload) && !payload.channel.isEmpty()) {
    *channel = payload.channel;
    return true;
  }
  return false;
}

}  // namespace

AudioPanel::AudioPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultAudioPanelConfig()), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setAcceptDrops(true);
  ApplyPanelShell(this);

  playback_ = new AudioPlaybackEngine(this);

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
  settings_widget_ = new AudioSettingsWidget(manager_, settings_scroll_);
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

  waveform_ = new AudioWaveformWidget(this);
  root->addWidget(waveform_, 1);

  connect(settings_widget_, &AudioSettingsWidget::configChanged, this, [this]() {
    const QString previous_channel = config_.channel;
    config_ = settings_widget_->config();
    applyConfigToUi();
    if (config_.channel != previous_channel) {
      timeline_buffer_.clear();
      configured_sample_rate_ = 0;
      configured_channels_ = 0;
      playback_->stop();
      resubscribeChannel();
    }
    emit configChanged();
  });
  connect(waveform_, &AudioWaveformWidget::seekRequested, this,
          &AudioPanel::onWaveformSeek);

  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  updateStatusBar();
  resubscribeChannel();
}

AudioPanel::~AudioPanel() { unsubscribeChannel(); }

void AudioPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("AudioDock");
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

AudioPanelConfig AudioPanel::config() const { return config_; }

void AudioPanel::setConfig(const AudioPanelConfig& config) {
  config_ = config;
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  resubscribeChannel();
}

void AudioPanel::cloneConfigFrom(const AudioPanelConfig& config) {
  setConfig(config);
}

void AudioPanel::setSettingsVisible(bool visible) {
  if (settings_container_ != nullptr) {
    settings_container_->setVisible(visible);
  }
  syncSettingsWidgetFromConfig();
}

bool AudioPanel::settingsVisible() const {
  return settings_container_ != nullptr && settings_container_->isVisible();
}

void AudioPanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ != nullptr) {
    settings_button_->blockSignals(true);
    settings_button_->setChecked(checked);
    settings_button_->blockSignals(false);
  }
}

void AudioPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ != nullptr) {
    expand_button_->blockSignals(true);
    expand_button_->setChecked(checked);
    expand_button_->blockSignals(false);
  }
}

QWidget* AudioPanel::settingsWidgetForInspector() {
  return SettingsScrollForInspector(settings_scroll_);
}

void AudioPanel::recallSettingsWidget() {
  RecallSettingsScrollToContainer(settings_scroll_, settings_container_);
}

void AudioPanel::refreshSettingsChannels() {
  if (settings_widget_ != nullptr) {
    settings_widget_->refreshChannels();
  }
}

void AudioPanel::handleChannelDrop(const QString& channel) {
  if (channel.isEmpty()) {
    return;
  }
  config_.channel = channel;
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
  timeline_buffer_.clear();
  configured_sample_rate_ = 0;
  configured_channels_ = 0;
  playback_->stop();
  resubscribeChannel();
  emit configChanged();
  emit activated();
}

void AudioPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void AudioPanel::dragEnterEvent(QDragEnterEvent* event) {
  if (event == nullptr) {
    return;
  }
  QString channel;
  if (ReadDropChannel(event->mimeData(), &channel)) {
    event->acceptProposedAction();
    emit activated();
  }
}

void AudioPanel::dragMoveEvent(QDragMoveEvent* event) {
  if (event == nullptr) {
    return;
  }
  QString channel;
  if (ReadDropChannel(event->mimeData(), &channel)) {
    event->acceptProposedAction();
  }
}

void AudioPanel::dropEvent(QDropEvent* event) {
  if (event == nullptr) {
    return;
  }
  QString channel;
  if (!ReadDropChannel(event->mimeData(), &channel)) {
    return;
  }
  handleChannelDrop(channel);
  event->acceptProposedAction();
}

void AudioPanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

QString AudioPanel::messageTypeForChannel(const QString& channel) const {
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

void AudioPanel::resubscribeChannel() {
  unsubscribeChannel();
  if (config_.channel.isEmpty()) {
    updateStatusBar();
    return;
  }
  subscribed_message_type_ = messageTypeForChannel(config_.channel).toStdString();
  subscription_id_ = integration::ChannelReaderRegistry::instance().subscribe(
      config_.channel.toStdString(),
      [this](const std::string& payload) { onChannelPayload(payload); });
  updateStatusBar();
}

void AudioPanel::unsubscribeChannel() {
  if (subscription_id_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(subscription_id_);
    subscription_id_ = 0;
  }
  subscribed_message_type_.clear();
}

void AudioPanel::onChannelPayload(const std::string& payload) {
  ingestPayload(payload);
}

void AudioPanel::ingestPayload(const std::string& payload) {
  ParsedRawAudio audio;
  std::string error_message;
  if (!ParseRawAudioPayload(subscribed_message_type_, payload, &audio,
                            &error_message)) {
    QMetaObject::invokeMethod(
        this,
        [this, error_message]() {
          status_label_->setText(QString::fromStdString(error_message));
        },
        Qt::QueuedConnection);
    return;
  }

  if (configured_sample_rate_ != audio.sample_rate ||
      configured_channels_ != audio.number_of_channels) {
    configured_sample_rate_ = audio.sample_rate;
    configured_channels_ = audio.number_of_channels;
    playback_->stop();
    playback_->configure(configured_sample_rate_, configured_channels_);
  }
  playback_->setVolume(config_.volume);
  playback_->setMuted(config_.mute);
  if (!config_.mute && !audio.pcm_samples.empty()) {
    playback_->enqueueSamples(audio.pcm_samples.data(), audio.pcm_samples.size());
  }

  timeline_buffer_.append(audio.timestamp_ns, audio.sample_rate,
                          audio.number_of_channels, std::move(audio.pcm_samples));
  timeline_buffer_.trimToWindow(
      static_cast<std::int64_t>(config_.window_size_sec * 1e9));

  playhead_ns_ = audio.timestamp_ns;

  QMetaObject::invokeMethod(this, "onRefreshUi", Qt::QueuedConnection);
}

void AudioPanel::onRefreshUi() {
  waveform_->setTimelineBuffer(&timeline_buffer_);
  waveform_->setWaveformColor(config_.waveform_color);
  waveform_->setPlayheadTimestampNs(playhead_ns_);
  updateStatusBar();
}

void AudioPanel::onWaveformSeek(std::int64_t timestamp_ns) {
  playhead_ns_ = timestamp_ns;
  waveform_->setPlayheadTimestampNs(playhead_ns_);
  if (manager_ != nullptr) {
    integration::PlaybackController& playback = manager_->playback();
    if (!playback.currentFile().empty()) {
      playback.seekTo(static_cast<double>(timestamp_ns) / 1e9);
      return;
    }
  }

  if (configured_sample_rate_ <= 0 || configured_channels_ <= 0 ||
      config_.mute || timeline_buffer_.empty()) {
    return;
  }
  constexpr std::int64_t kPreviewSliceNs = 500000000;  // 500 ms
  std::vector<std::int16_t> preview_frames;
  if (!timeline_buffer_.readInterleavedFrames(timestamp_ns, timestamp_ns + kPreviewSliceNs,
                                              &preview_frames)) {
    return;
  }
  playback_->stop();
  playback_->configure(configured_sample_rate_, configured_channels_);
  playback_->setVolume(config_.volume);
  playback_->setMuted(config_.mute);
  playback_->enqueueSamples(preview_frames.data(), preview_frames.size());
}

void AudioPanel::applyConfigToUi() {
  playback_->setVolume(config_.volume);
  playback_->setMuted(config_.mute);
  waveform_->setWaveformColor(config_.waveform_color);
  waveform_->setDefaultWindowSec(config_.window_size_sec);
  timeline_buffer_.trimToWindow(
      static_cast<std::int64_t>(config_.window_size_sec * 1e9));
  waveform_->setTimelineBuffer(&timeline_buffer_);
}

void AudioPanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ != nullptr) {
    settings_widget_->setConfig(config_);
  }
}

void AudioPanel::updateStatusBar() {
  if (status_label_ == nullptr) {
    return;
  }
  if (config_.channel.isEmpty()) {
    status_label_->setText(tr("Select a RawAudio channel in settings or drag from Channels"));
    return;
  }
  const QString type =
      subscribed_message_type_.empty()
          ? tr("unknown type")
          : QString::fromStdString(subscribed_message_type_);
  if (timeline_buffer_.empty()) {
    status_label_->setText(tr("Waiting for %1 on %2")
                               .arg(type, config_.channel));
    return;
  }
  QString status =
      tr("%1 • %2 Hz • %3 ch • window %4 s")
          .arg(config_.channel)
          .arg(timeline_buffer_.sampleRate())
          .arg(timeline_buffer_.channelCount())
          .arg(config_.window_size_sec, 0, 'f', 1);
  if (playback_ != nullptr && !playback_->playbackAvailable()) {
    status += tr(" • playback unavailable (install Qt6 Multimedia dev package)");
  }
  status_label_->setText(status);
}

}  // namespace audio_panel
}  // namespace autoviz

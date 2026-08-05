/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QPointer>
#include <QWidget>

#include "autoviz/ui/audio/audio_timeline_buffer.hpp"
#include "autoviz/ui/audio/audio_types.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"

class QFrame;
class QFocusEvent;
class QDragEnterEvent;
class QDragMoveEvent;
class QDropEvent;
class QLabel;
class QScrollArea;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace audio_panel {

class AudioPlaybackEngine;
class AudioSettingsWidget;
class AudioWaveformWidget;

class AudioPanel : public QWidget {
  Q_OBJECT

 public:
  explicit AudioPanel(common::VisualizationManager* manager,
                      QWidget* parent = nullptr);
  ~AudioPanel() override;

  void installTitleBarTools(PanelDockWidget* dock);

  AudioPanelConfig config() const;
  void setConfig(const AudioPanelConfig& config);
  void cloneConfigFrom(const AudioPanelConfig& config);
  void setSettingsVisible(bool visible);
  bool settingsVisible() const;
  void setSettingsButtonChecked(bool checked);
  void setExpandButtonChecked(bool checked);
  QWidget* settingsWidgetForInspector();
  void recallSettingsWidget();
  void refreshSettingsChannels();
  void handleChannelDrop(const QString& channel);

 signals:
  void configChanged();
  void activated();
  void settingsToggled(bool visible);
  void panelSplitRequested(Qt::Orientation orientation);
  void panelExpandRequested();
  void panelRemoveRequested();
  void panelChangeRequested(const QString& object_name);

 protected:
  void focusInEvent(QFocusEvent* event) override;
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dragMoveEvent(QDragMoveEvent* event) override;
  void dropEvent(QDropEvent* event) override;

 private slots:
  void onToggleSettings(bool visible);
  void onChannelPayload(const std::string& payload);
  void onWaveformSeek(std::int64_t timestamp_ns);
  void onRefreshUi();

 private:
  void resubscribeChannel();
  void unsubscribeChannel();
  void applyConfigToUi();
  void syncSettingsWidgetFromConfig();
  void updateStatusBar();
  void ingestPayload(const std::string& payload);
  QString messageTypeForChannel(const QString& channel) const;

  common::VisualizationManager* manager_ = nullptr;
  AudioPanelConfig config_;
  AudioTimelineBuffer timeline_buffer_;
  AudioWaveformWidget* waveform_ = nullptr;
  AudioPlaybackEngine* playback_ = nullptr;
  QLabel* status_label_ = nullptr;
  AudioSettingsWidget* settings_widget_ = nullptr;
  QScrollArea* settings_scroll_ = nullptr;
  QWidget* settings_container_ = nullptr;
  QPointer<QToolButton> settings_button_;
  QPointer<QToolButton> expand_button_;
  integration::ChannelReaderRegistry::SubscriptionId subscription_id_ = 0;
  std::string subscribed_message_type_;
  std::int64_t playhead_ns_ = 0;
  int configured_sample_rate_ = 0;
  int configured_channels_ = 0;
};

}  // namespace audio_panel
}  // namespace autoviz

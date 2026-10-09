/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file record_panel.hpp
 * @brief Record playback panel (Foxglove MCAP + rqt_bag inspired).
 *
 * Docked in the right sidebar. Combines Foxglove-style transport (play / step /
 * seek / loop / rate / trim range) with rqt_bag-style channel mute list,
 * message stepping, and filter.
 *
 * @see integration::PlaybackController
 * @see FrameSession::openRecordFile()
 */

#pragma once

#include <QWidget>

#include <set>
#include <string>
#include <vector>

class QCheckBox;
class QComboBox;
class QKeyEvent;
class QLabel;
class QLineEdit;
class QSlider;
class QTimer;
class QToolButton;
class QTreeWidget;
class QTreeWidgetItem;

namespace autoviz {
namespace common {
class VisualizationManager;
}
class PanelDockWidget;

/**
 * @class RecordPanel
 * @brief Autolink .record / .bag / .mcap player UI.
 *
 * Shortcuts (panel focus): Space play/pause, Left/Right ±100 ms,
 * Alt+Left/Right message step, Home/End start/end of active range.
 */
class RecordPanel : public QWidget {
  Q_OBJECT

 public:
  explicit RecordPanel(common::VisualizationManager* manager,
                       QWidget* parent = nullptr);
  ~RecordPanel() override = default;

  void installTitleBarTools(PanelDockWidget* dock);
  void setExpandButtonChecked(bool checked);

  /** @brief Rebuild channel list / labels from the current playback file. */
  void reloadFromPlayback();

 signals:
  void openRecordRequested();
  void openInRawMessagesRequested(const QString& channel);
  void panelRemoveRequested();
  void panelExpandRequested();
  void panelSplitRequested(Qt::Orientation orientation);
  void panelChangeRequested(const QString& object_name);

 protected:
  void keyPressEvent(QKeyEvent* event) override;

 private slots:
  void onOpenClicked();
  void onPlayPauseClicked();
  void onStopClicked();
  void onSeekStartClicked();
  void onSeekEndClicked();
  void onStepBackClicked();
  void onStepForwardClicked();
  void onStepPrevMessage();
  void onStepNextMessage();
  void onSetRangeStart();
  void onSetRangeEnd();
  void onClearRange();
  void onLoopToggled(bool checked);
  void onRateChanged(int index);
  void onSeekPressed();
  void onSeekReleased();
  void onSeekMoved(int value);
  void onChannelItemChanged(QTreeWidgetItem* item, int column);
  void onChannelDoubleClicked(QTreeWidgetItem* item, int column);
  void onFilterTextChanged(const QString& text);
  void onSelectAllChannels();
  void onSelectNoneChannels();
  void onTick();

 private:
  class DensityBar;

  void applyChromeStyles();
  void syncTransportButtons();
  void syncTimelineFromPlayback();
  void syncRangeLabel();
  void syncDensityBar();
  void applyExcludedChannelsFromTree();
  void rebuildChannelTree();
  void applyChannelFilter();
  void seekByDelta(double delta_sec);
  void seekToNormalized(double normalized);
  bool ensureFileOpen();
  std::string selectedChannel() const;
  static QString FormatClock(double seconds);
  static QString ShortMessageType(const std::string& message_type);

  common::VisualizationManager* manager_ = nullptr;

  QToolButton* open_button_ = nullptr;
  QToolButton* seek_start_button_ = nullptr;
  QToolButton* step_back_button_ = nullptr;
  QToolButton* play_pause_button_ = nullptr;
  QToolButton* step_forward_button_ = nullptr;
  QToolButton* seek_end_button_ = nullptr;
  QToolButton* stop_button_ = nullptr;
  QToolButton* prev_msg_button_ = nullptr;
  QToolButton* next_msg_button_ = nullptr;
  QCheckBox* loop_check_ = nullptr;
  QComboBox* rate_combo_ = nullptr;
  QLabel* file_label_ = nullptr;
  QLabel* time_label_ = nullptr;
  DensityBar* density_bar_ = nullptr;
  QSlider* timeline_ = nullptr;
  QToolButton* range_in_button_ = nullptr;
  QToolButton* range_out_button_ = nullptr;
  QToolButton* range_clear_button_ = nullptr;
  QLabel* range_label_ = nullptr;
  QLineEdit* filter_edit_ = nullptr;
  QToolButton* select_all_button_ = nullptr;
  QToolButton* select_none_button_ = nullptr;
  QLabel* channels_header_ = nullptr;
  QTreeWidget* channel_tree_ = nullptr;
  QTimer* tick_timer_ = nullptr;
  QToolButton* expand_button_ = nullptr;

  bool seeking_ = false;
  bool scrub_resume_play_ = false;
  qint64 last_scrub_ms_ = 0;
  bool rebuilding_channels_ = false;
  std::string loaded_file_;
};

}  // namespace autoviz

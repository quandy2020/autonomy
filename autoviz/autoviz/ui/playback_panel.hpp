/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QWidget>

namespace autoviz {
namespace integration {
class PlaybackController;
}

class PlaybackPanel : public QWidget {
  Q_OBJECT

 public:
  explicit PlaybackPanel(integration::PlaybackController* controller,
                         QWidget* parent = nullptr);

 private slots:
  void onOpenRecord();
  void onImport();
  void onPlay();
  void onPause();
  void onStop();
  void onRefreshProgress();
  void onSeekSliderPressed();
  void onSeekSliderReleased();
  void onSeekSliderMoved(int value);
  void onStepBackward();
  void onStepForward();
  void onSyncRate();

 private:
  void setupUi();
  void updateUi();

  integration::PlaybackController* controller_ = nullptr;
  QLabel* file_label_ = nullptr;
  QLabel* time_label_ = nullptr;
  QLabel* status_label_ = nullptr;
  QLabel* channels_label_ = nullptr;
  QSlider* seek_slider_ = nullptr;
  QDoubleSpinBox* rate_spin_ = nullptr;
  QCheckBox* loop_check_ = nullptr;
  QPushButton* play_button_ = nullptr;
  QPushButton* pause_button_ = nullptr;
  QPushButton* stop_button_ = nullptr;
  QTimer refresh_timer_;
  bool seeking_ = false;
};

}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/playback_panel.hpp"

#include <algorithm>

#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QVBoxLayout>

#include "autoviz/integration/playback_controller.hpp"
#include "autoviz/ui/import_record_dialog.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/record_open_utils.hpp"

namespace autoviz {

PlaybackPanel::PlaybackPanel(integration::PlaybackController* controller,
                             QWidget* parent)
    : QWidget(parent), controller_(controller) {
  setupUi();
  updateUi();
}

void PlaybackPanel::setupUi() {
  ApplyPanelShell(this);
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  QHBoxLayout* toolbar_layout = nullptr;
  auto* toolbar = MakePanelToolbar(this, &toolbar_layout);
  file_label_ = new QLabel(tr("No record loaded"), toolbar);
  file_label_->setWordWrap(true);
  toolbar_layout->addWidget(file_label_, 1);
  status_label_ = new QLabel(tr("Stopped"), toolbar);
  StyleHintLabel(status_label_);
  toolbar_layout->addWidget(status_label_);
  layout->addWidget(toolbar);

  auto* body = new QWidget(this);
  auto* body_layout = new QVBoxLayout(body);
  ApplyCompactVBox(body_layout);

  channels_label_ = new QLabel(tr("Channels: —"), body);
  channels_label_->setWordWrap(true);
  StyleHintLabel(channels_label_);
  body_layout->addWidget(channels_label_);

  seek_slider_ = new QSlider(Qt::Horizontal, body);
  seek_slider_->setRange(0, 1000);
  seek_slider_->setEnabled(false);
  body_layout->addWidget(seek_slider_);

  time_label_ = new QLabel(tr("Sim Time: 0.000 / 0.000 s"), body);
  body_layout->addWidget(time_label_);

  auto* sync_label = new QLabel(tr("Time Source: Sim Time"), body);
  StyleHintLabel(sync_label);
  body_layout->addWidget(sync_label);

  auto* step_row = new QHBoxLayout();
  auto* step_back = new QPushButton(tr("−0.1s"), body);
  auto* step_forward = new QPushButton(tr("+0.1s"), body);
  step_row->addWidget(step_back);
  step_row->addWidget(step_forward);
  body_layout->addLayout(step_row);

  auto* rate_row = new QHBoxLayout();
  rate_row->addWidget(new QLabel(tr("Rate"), body));
  rate_spin_ = new QDoubleSpinBox(body);
  rate_spin_->setRange(0.1, 8.0);
  rate_spin_->setSingleStep(0.1);
  rate_spin_->setValue(1.0);
  rate_row->addWidget(rate_spin_);
  loop_check_ = new QCheckBox(tr("Loop"), body);
  rate_row->addWidget(loop_check_);
  body_layout->addLayout(rate_row);

  auto* button_row = new QHBoxLayout();
  auto* open_button = new QPushButton(tr("Open..."), body);
  auto* import_button = new QPushButton(tr("Import..."), body);
  play_button_ = new QPushButton(tr("Play"), body);
  pause_button_ = new QPushButton(tr("Pause"), body);
  stop_button_ = new QPushButton(tr("Stop"), body);
  button_row->addWidget(open_button);
  button_row->addWidget(import_button);
  button_row->addWidget(play_button_);
  button_row->addWidget(pause_button_);
  button_row->addWidget(stop_button_);
  body_layout->addLayout(button_row);
  body_layout->addStretch();
  layout->addWidget(body, 1);

  connect(open_button, &QPushButton::clicked, this, &PlaybackPanel::onOpenRecord);
  connect(import_button, &QPushButton::clicked, this, &PlaybackPanel::onImport);
  connect(play_button_, &QPushButton::clicked, this, &PlaybackPanel::onPlay);
  connect(pause_button_, &QPushButton::clicked, this, &PlaybackPanel::onPause);
  connect(stop_button_, &QPushButton::clicked, this, &PlaybackPanel::onStop);
  connect(step_back, &QPushButton::clicked, this, &PlaybackPanel::onStepBackward);
  connect(step_forward, &QPushButton::clicked, this, &PlaybackPanel::onStepForward);
  connect(rate_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          &PlaybackPanel::onSyncRate);
  connect(seek_slider_, &QSlider::sliderPressed, this,
          &PlaybackPanel::onSeekSliderPressed);
  connect(seek_slider_, &QSlider::sliderReleased, this,
          &PlaybackPanel::onSeekSliderReleased);
  connect(seek_slider_, &QSlider::sliderMoved, this,
          &PlaybackPanel::onSeekSliderMoved);
  connect(&refresh_timer_, &QTimer::timeout, this,
          &PlaybackPanel::onRefreshProgress);
  refresh_timer_.start(200);
}

void PlaybackPanel::updateUi() {
  if (controller_ == nullptr) {
    return;
  }
  const QString file =
      controller_->currentFile().empty()
          ? tr("No record loaded")
          : QFileInfo(QString::fromStdString(controller_->currentFile()))
                .fileName();
  file_label_->setText(file);
  if (controller_->currentFile().empty()) {
    channels_label_->setText(tr("Channels: —"));
  } else {
    channels_label_->setText(
        tr("Channels: %1").arg(controller_->channelCount()));
  }
  seek_slider_->setEnabled(!controller_->currentFile().empty());
  play_button_->setEnabled(!controller_->currentFile().empty());
  pause_button_->setEnabled(controller_->isPlaying());
  stop_button_->setEnabled(controller_->isPlaying());
  pause_button_->setText(controller_->isPaused() ? tr("Resume") : tr("Pause"));
  if (controller_->isPlaying()) {
    status_label_->setText(controller_->isPaused() ? tr("Paused") : tr("Playing"));
  } else {
    status_label_->setText(tr("Stopped"));
  }
  rate_spin_->setValue(controller_->playRate());
  onRefreshProgress();
}

void PlaybackPanel::onRefreshProgress() {
  if (controller_ == nullptr || seeking_) {
    return;
  }
  const double total = controller_->totalTimeSec();
  const double current = controller_->currentTimeSec();
  const double progress = controller_->progress();
  seek_slider_->setValue(static_cast<int>(progress * 1000.0));
  time_label_->setText(
      tr("Sim Time: %1 / %2 s")
          .arg(current, 0, 'f', 3)
          .arg(total, 0, 'f', 3));
}

void PlaybackPanel::onSeekSliderPressed() {
  seeking_ = true;
}

void PlaybackPanel::onSeekSliderReleased() {
  if (controller_ == nullptr) {
    seeking_ = false;
    return;
  }
  const double total = controller_->totalTimeSec();
  const double target = total * (seek_slider_->value() / 1000.0);
  controller_->seekTo(target);
  seeking_ = false;
  updateUi();
}

void PlaybackPanel::onSeekSliderMoved(int value) {
  if (controller_ == nullptr) {
    return;
  }
  const double total = controller_->totalTimeSec();
  const double current = total * (static_cast<double>(value) / 1000.0);
  time_label_->setText(
      tr("Sim Time: %1 / %2 s")
          .arg(current, 0, 'f', 3)
          .arg(total, 0, 'f', 3));
}

void PlaybackPanel::onStepBackward() {
  if (controller_ == nullptr || controller_->currentFile().empty()) {
    return;
  }
  const double target =
      std::max(0.0, controller_->currentTimeSec() - 0.1);
  controller_->seekTo(target);
  updateUi();
}

void PlaybackPanel::onStepForward() {
  if (controller_ == nullptr || controller_->currentFile().empty()) {
    return;
  }
  const double target =
      std::min(controller_->totalTimeSec(), controller_->currentTimeSec() + 0.1);
  controller_->seekTo(target);
  updateUi();
}

void PlaybackPanel::onSyncRate() {
  if (controller_ == nullptr || !controller_->isPlaying() ||
      controller_->isPaused()) {
    return;
  }
  controller_->play(rate_spin_->value(), loop_check_->isChecked());
}

void PlaybackPanel::syncFromController() {
  updateUi();
}

bool PlaybackPanel::openAndPlay(const QString& path) {
  if (controller_ == nullptr) {
    return false;
  }
  const RecordSourceKind kind = ClassifyRecordSource(path);
  OpenRecordResult result = OpenRecordSource(controller_, path);
  if (!result.ok &&
      (kind == RecordSourceKind::kBag || kind == RecordSourceKind::kMcap)) {
    ImportRecordDialog dialog(controller_, this);
    dialog.setSourcePath(path);
    if (dialog.exec() != QDialog::Accepted || !dialog.recordOpened()) {
      updateUi();
      return false;
    }
  } else if (!result.ok) {
    QMessageBox::warning(this, tr("Open Record"), result.error);
    updateUi();
    return false;
  }
  controller_->play(rate_spin_->value(), loop_check_->isChecked());
  updateUi();
  return true;
}

void PlaybackPanel::onOpenRecord() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Open Record"), QString(),
      tr("Autolink Record (*.record);;Legacy Bag (*.bag);;MCAP (*.mcap);;All Files (*)"));
  if (path.isEmpty()) {
    return;
  }
  openAndPlay(path);
}

void PlaybackPanel::onImport() {
  ImportRecordDialog dialog(controller_, this);
  if (dialog.exec() == QDialog::Accepted && dialog.recordOpened()) {
    controller_->play(rate_spin_->value(), loop_check_->isChecked());
    updateUi();
  }
}

void PlaybackPanel::onPlay() {
  controller_->play(rate_spin_->value(), loop_check_->isChecked());
  updateUi();
}

void PlaybackPanel::onPause() {
  if (controller_->isPaused()) {
    controller_->resume();
  } else {
    controller_->pause();
  }
  updateUi();
}

void PlaybackPanel::onStop() {
  controller_->stop();
  updateUi();
}

}  // namespace autoviz

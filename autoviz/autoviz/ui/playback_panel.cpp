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

namespace autoviz {

PlaybackPanel::PlaybackPanel(integration::PlaybackController* controller,
                             QWidget* parent)
    : QWidget(parent), controller_(controller) {
  setupUi();
  updateUi();
}

void PlaybackPanel::setupUi() {
  auto* layout = new QVBoxLayout(this);

  file_label_ = new QLabel(tr("No record loaded"), this);
  file_label_->setWordWrap(true);
  layout->addWidget(file_label_);

  status_label_ = new QLabel(tr("Stopped"), this);
  layout->addWidget(status_label_);

  channels_label_ = new QLabel(tr("Channels: —"), this);
  channels_label_->setWordWrap(true);
  layout->addWidget(channels_label_);

  seek_slider_ = new QSlider(Qt::Horizontal, this);
  seek_slider_->setRange(0, 1000);
  seek_slider_->setEnabled(false);
  layout->addWidget(seek_slider_);

  time_label_ = new QLabel(tr("Sim Time: 0.000 / 0.000 s"), this);
  layout->addWidget(time_label_);

  auto* sync_label = new QLabel(tr("Time Source: Sim Time"), this);
  sync_label->setStyleSheet(QStringLiteral("color: palette(mid);"));
  layout->addWidget(sync_label);

  auto* step_row = new QHBoxLayout();
  auto* step_back = new QPushButton(tr("−0.1s"), this);
  auto* step_forward = new QPushButton(tr("+0.1s"), this);
  step_row->addWidget(step_back);
  step_row->addWidget(step_forward);
  layout->addLayout(step_row);

  auto* rate_row = new QHBoxLayout();
  rate_row->addWidget(new QLabel(tr("Rate"), this));
  rate_spin_ = new QDoubleSpinBox(this);
  rate_spin_->setRange(0.1, 8.0);
  rate_spin_->setSingleStep(0.1);
  rate_spin_->setValue(1.0);
  rate_row->addWidget(rate_spin_);
  loop_check_ = new QCheckBox(tr("Loop"), this);
  rate_row->addWidget(loop_check_);
  layout->addLayout(rate_row);

  auto* button_row = new QHBoxLayout();
  auto* open_button = new QPushButton(tr("Open..."), this);
  auto* import_button = new QPushButton(tr("Import..."), this);
  play_button_ = new QPushButton(tr("Play"), this);
  pause_button_ = new QPushButton(tr("Pause"), this);
  stop_button_ = new QPushButton(tr("Stop"), this);
  button_row->addWidget(open_button);
  button_row->addWidget(import_button);
  button_row->addWidget(play_button_);
  button_row->addWidget(pause_button_);
  button_row->addWidget(stop_button_);
  layout->addLayout(button_row);

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

void PlaybackPanel::onOpenRecord() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Open Autolink Record"), QString(),
      tr("Autolink Record (*.record);;All Files (*)"));
  if (path.isEmpty()) {
    return;
  }
  const QString suffix = QFileInfo(path).suffix().toLower();
  if (suffix == QLatin1String("bag") || suffix == QLatin1String("mcap")) {
    QMessageBox::information(
        this, tr("Convert Required"),
        tr("Autoviz replays Autolink .record files.\n\n"
           "Convert legacy bag offline:\n"
           "  bag_to_record input.bag output.record\n\n"
           "Then open output.record in Playback."));
    return;
  }
  if (!controller_->openFile(path.toStdString())) {
    QMessageBox::warning(
        this, tr("Open Failed"),
        tr("Could not open record file:\n%1\n\n"
           "Ensure the file is a valid Autolink .record.")
            .arg(path));
    updateUi();
    return;
  }
  updateUi();
}

void PlaybackPanel::onImport() {
  ImportRecordDialog dialog(controller_, this);
  if (dialog.exec() == QDialog::Accepted && dialog.recordOpened()) {
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

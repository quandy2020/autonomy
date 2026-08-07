/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/time_panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"

namespace autoviz {

TimePanel::TimePanel(common::VisualizationManager* manager, QWidget* parent)
    : QWidget(parent), manager_(manager) {
  sim_time_label_ = makeTimeLabel();
  sim_elapsed_label_ = makeTimeLabel();
  wall_time_label_ = makeTimeLabel();
  wall_elapsed_label_ = makeTimeLabel();

  experimental_cb_ = new QCheckBox(tr("Experimental"), this);
  experimental_cb_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

  pause_button_ = new QPushButton(tr("Pause"), this);
  pause_button_->setToolTip(tr("Freeze sim time."));
  pause_button_->setCheckable(true);

  reset_button_ = new QPushButton(tr("Reset"), this);
  reset_button_->setToolTip(
      tr("Reset sim time, displays, and TF cache (shortcut: R)."));
  connect(reset_button_, &QPushButton::clicked, this,
          &TimePanel::resetRequested);

  sync_mode_selector_ = new QComboBox(this);
  sync_mode_selector_->addItem(tr("Off"));
  sync_mode_selector_->addItem(tr("Exact"));
  sync_mode_selector_->addItem(tr("Approximate"));
  sync_mode_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  sync_mode_selector_->setToolTip(
      tr("Synchronize sim time and TF transforms to a given source."));

  sync_source_selector_ = new QComboBox(this);
  sync_source_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  sync_source_selector_->setToolTip(tr("Time source to use for synchronization."));

  experimental_widget_ = new QWidget(this);
  auto* experimental_layout = new QHBoxLayout(experimental_widget_);
  experimental_layout->setContentsMargins(0, 0, 20, 0);
  experimental_layout->addWidget(pause_button_);
  experimental_layout->addWidget(new QLabel(tr("Synchronization:"), this));
  experimental_layout->addWidget(sync_mode_selector_);
  experimental_layout->addWidget(new QLabel(tr("Source:"), this));
  experimental_layout->addWidget(sync_source_selector_);
  experimental_layout->addSpacing(20);

  old_widget_ = new QWidget(this);
  auto* old_layout = new QHBoxLayout(old_widget_);
  old_layout->setContentsMargins(0, 0, 20, 0);
  old_layout->addWidget(new QLabel(tr("Elapsed:"), this));
  old_layout->addWidget(sim_elapsed_label_);
  old_layout->addWidget(new QLabel(tr("Wall Time:"), this));
  old_layout->addWidget(wall_time_label_);
  old_layout->addWidget(new QLabel(tr("Wall Elapsed:"), this));
  old_layout->addWidget(wall_elapsed_label_);

  fps_label_ = new QLabel(this);
  fps_label_->setMinimumWidth(48);
  fps_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  bottom_row_ = new QWidget(this);
  auto* bottom_layout = new QHBoxLayout(bottom_row_);
  bottom_layout->setContentsMargins(0, 0, 0, 0);
  bottom_layout->addWidget(reset_button_);
  bottom_layout->addStretch(100);
  bottom_layout->addWidget(experimental_cb_);
  bottom_layout->addWidget(fps_label_);

  auto* top_row = new QWidget(this);
  auto* top_layout = new QHBoxLayout(top_row);
  top_layout->setContentsMargins(0, 0, 0, 0);
  top_layout->addWidget(experimental_widget_);
  top_layout->addWidget(new QLabel(tr("Time:"), this));
  top_layout->addWidget(sim_time_label_);
  top_layout->addWidget(old_widget_);
  top_layout->addStretch(100);

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(11, 5, 11, 5);
  layout->setSpacing(4);
  layout->addWidget(top_row);
  layout->addWidget(bottom_row_);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  connect(experimental_cb_, &QCheckBox::toggled, this,
          &TimePanel::experimentalToggled);
  connect(pause_button_, &QPushButton::toggled, this, &TimePanel::pauseToggled);
  connect(sync_mode_selector_, qOverload<int>(&QComboBox::activated), this,
          &TimePanel::syncModeSelected);
  connect(sync_source_selector_, qOverload<int>(&QComboBox::activated), this,
          &TimePanel::syncSourceSelected);

  auto* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &TimePanel::update);
  timer->start(100);

  experimentalToggled(false);
  syncModeSelected(0);
  pauseToggled(false);
}

bool TimePanel::experimental() const {
  return experimental_cb_->isChecked();
}

int TimePanel::syncMode() const { return sync_mode_selector_->currentIndex(); }

QString TimePanel::syncSource() const {
  return sync_source_selector_->currentText();
}

void TimePanel::setExperimental(bool enabled) {
  experimental_cb_->setChecked(enabled);
  experimentalToggled(enabled);
}

void TimePanel::setSyncMode(int mode) {
  if (mode >= 0 && mode < sync_mode_selector_->count()) {
    sync_mode_selector_->setCurrentIndex(mode);
    syncModeSelected(mode);
  }
}

void TimePanel::setSyncSource(const QString& source) {
  config_sync_source_ = source;
  refreshSyncSources();
  const int index = sync_source_selector_->findText(source);
  if (index >= 0) {
    sync_source_selector_->setCurrentIndex(index);
  }
}

void TimePanel::setFpsText(const QString& text) { fps_label_->setText(text); }

QLineEdit* TimePanel::makeTimeLabel() {
  auto* label = new QLineEdit(this);
  label->setReadOnly(true);
  label->setMinimumWidth(88);
  label->setMaximumWidth(120);
  label->setFrame(true);
  return label;
}

void TimePanel::fillTimeLabel(QLineEdit* label, double time) {
  label->setText(QString::number(time, 'f', 2));
}

void TimePanel::notifyLayoutChanged() {
  updateGeometry();
  emit layoutChanged();
}

void TimePanel::refreshSyncSources() {
  if (manager_ == nullptr) {
    return;
  }
  const QString current = sync_source_selector_->currentText();
  sync_source_selector_->blockSignals(true);
  sync_source_selector_->clear();
  for (const auto* display : manager_->displays()) {
    if (display == nullptr) {
      continue;
    }
    sync_source_selector_->addItem(QString::fromStdString(display->name()));
  }
  if (!config_sync_source_.isEmpty()) {
    const int index = sync_source_selector_->findText(config_sync_source_);
    if (index >= 0) {
      sync_source_selector_->setCurrentIndex(index);
      config_sync_source_.clear();
    }
  } else if (!current.isEmpty()) {
    const int index = sync_source_selector_->findText(current);
    if (index >= 0) {
      sync_source_selector_->setCurrentIndex(index);
    }
  }
  sync_source_selector_->blockSignals(false);
}

void TimePanel::update() {
  if (manager_ == nullptr) {
    return;
  }
  if (experimental_cb_->isChecked()) {
    refreshSyncSources();
  }
  fillTimeLabel(sim_time_label_, manager_->simTimeSec());
  fillTimeLabel(sim_elapsed_label_, manager_->simTimeElapsedSec());
  fillTimeLabel(wall_time_label_, manager_->wallClockSec());
  fillTimeLabel(wall_elapsed_label_, manager_->wallClockElapsedSec());
}

void TimePanel::pauseToggled(bool checked) {
  if (manager_ != nullptr) {
    manager_->setTimePaused(checked);
  }
}

void TimePanel::experimentalToggled(bool checked) {
  old_widget_->setVisible(!checked);
  experimental_widget_->setVisible(checked);
  if (manager_ == nullptr) {
    notifyLayoutChanged();
    return;
  }
  if (!checked) {
    pause_button_->setChecked(false);
    pauseToggled(false);
    sync_mode_selector_->setCurrentIndex(0);
    syncModeSelected(0);
  } else {
    pauseToggled(pause_button_->isChecked());
    syncModeSelected(sync_mode_selector_->currentIndex());
    refreshSyncSources();
  }
  notifyLayoutChanged();
}

void TimePanel::syncSourceSelected(int /*index*/) {
  config_sync_source_.clear();
  if (manager_ != nullptr) {
    manager_->setTimeSyncSource(sync_source_selector_->currentText().toStdString());
  }
}

void TimePanel::syncModeSelected(int mode) {
  if (manager_ != nullptr) {
    manager_->setTimeSyncMode(static_cast<common::TimeSyncMode>(mode));
  }
  sync_source_selector_->setEnabled(mode != 0);
  notifyLayoutChanged();
}

}  // namespace autoviz

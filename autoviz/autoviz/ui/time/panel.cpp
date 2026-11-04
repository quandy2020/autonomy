/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/time/panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSizePolicy>
#include <QTimer>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

QWidget* MakeMetricChip(const QString& caption, QLineEdit* value,
                        QWidget* parent) {
  auto* chip = new QFrame(parent);
  chip->setObjectName(QStringLiteral("TimeMetricChip"));
  chip->setAttribute(Qt::WA_StyledBackground, true);
  chip->setStyleSheet(style::sheet(QStringLiteral("time/metric_chip"),
                                   style::tokens()));
  auto* layout = new QHBoxLayout(chip);
  layout->setContentsMargins(10, 4, 10, 4);
  layout->setSpacing(8);
  auto* label = MakeSettingsFormLabel(caption, chip);
  label->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
  value->setParent(chip);
  layout->addWidget(label, 0);
  layout->addWidget(value, 1);
  chip->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  return chip;
}

}  // namespace

TimePanel::TimePanel(common::VisualizationManager* manager, QWidget* parent)
    : QWidget(parent), manager_(manager) {
  ApplyPanelShell(this);
  setObjectName(QStringLiteral("Time/TimePanel"));

  sim_time_label_ = makeTimeLabel();
  sim_elapsed_label_ = makeTimeLabel();
  wall_time_label_ = makeTimeLabel();
  wall_elapsed_label_ = makeTimeLabel();

  experimental_cb_ = new QCheckBox(tr("Experimental"), this);
  experimental_cb_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  experimental_cb_->setStyleSheet(
      style::sheet(QStringLiteral("time/check"), style::tokens()));

  pause_button_ = MakeFlatActionButton(tr("Pause"), this);
  pause_button_->setToolTip(tr("Freeze sim time."));
  pause_button_->setCheckable(true);

  reset_button_ = MakeFlatActionButton(tr("Reset"), this);
  reset_button_->setAutoDefault(false);
  reset_button_->setDefault(false);
  reset_button_->setFocusPolicy(Qt::NoFocus);
  reset_button_->setToolTip(
      tr("Reset elapsed time, displays, and TF cache (shortcut: R)."));
  connect(reset_button_, &QPushButton::clicked, this, &TimePanel::onResetClicked);

  sync_mode_selector_ = new QComboBox(this);
  sync_mode_selector_->addItem(tr("Off"));
  sync_mode_selector_->addItem(tr("Exact"));
  sync_mode_selector_->addItem(tr("Approximate"));
  sync_mode_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  sync_mode_selector_->setToolTip(
      tr("Synchronize sim time and TF transforms to a given source."));
  StyleCompactSettingsField(sync_mode_selector_);

  sync_source_selector_ = new QComboBox(this);
  sync_source_selector_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  sync_source_selector_->setToolTip(tr("Time source to use for synchronization."));
  StyleCompactSettingsField(sync_source_selector_);

  experimental_widget_ = new QWidget(this);
  auto* experimental_layout = new QHBoxLayout(experimental_widget_);
  experimental_layout->setContentsMargins(0, 0, 0, 0);
  experimental_layout->setSpacing(8);
  experimental_layout->addWidget(pause_button_);
  experimental_layout->addWidget(MakeSettingsFormLabel(tr("Sync"), experimental_widget_));
  experimental_layout->addWidget(sync_mode_selector_);
  experimental_layout->addWidget(MakeSettingsFormLabel(tr("Source"), experimental_widget_));
  experimental_layout->addWidget(sync_source_selector_);
  experimental_widget_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);

  // Absolute sim time stays visible in both layouts (matches prior chrome).
  auto* time_chip = MakeMetricChip(tr("Time"), sim_time_label_, this);

  old_widget_ = new QWidget(this);
  auto* metrics_layout = new QHBoxLayout(old_widget_);
  metrics_layout->setContentsMargins(0, 0, 0, 0);
  metrics_layout->setSpacing(8);
  metrics_layout->addWidget(
      MakeMetricChip(tr("Elapsed"), sim_elapsed_label_, old_widget_), 1);
  metrics_layout->addWidget(
      MakeMetricChip(tr("Wall Time"), wall_time_label_, old_widget_), 1);
  metrics_layout->addWidget(
      MakeMetricChip(tr("Wall Elapsed"), wall_elapsed_label_, old_widget_), 1);
  old_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

  fps_label_ = new QLabel(this);
  fps_label_->setMinimumWidth(56);
  fps_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  StyleHintLabel(fps_label_);

  QHBoxLayout* strip_layout = nullptr;
  auto* strip = MakePanelToolbar(this, &strip_layout);
  strip_layout->setContentsMargins(PanelChromeLayout::kToolbarMarginH, 6,
                                   PanelChromeLayout::kToolbarMarginH, 6);
  strip_layout->setSpacing(10);
  strip_layout->addWidget(time_chip, 1);
  strip_layout->addWidget(old_widget_, 3);
  strip_layout->addWidget(experimental_widget_, 0);
  strip_layout->addStretch(1);
  strip_layout->addWidget(reset_button_, 0, Qt::AlignVCenter);
  strip_layout->addWidget(experimental_cb_, 0, Qt::AlignVCenter);
  strip_layout->addWidget(fps_label_, 0, Qt::AlignVCenter);

  // Keep bottom_row_ for API compatibility / geometry; fold into the strip.
  bottom_row_ = new QWidget(this);
  bottom_row_->hide();
  bottom_row_->setFixedHeight(0);

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  layout->addWidget(strip);
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
  connect(timer, &QTimer::timeout, this, &TimePanel::refreshTimes);
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
  label->setFrame(false);
  label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  label->setMinimumWidth(96);
  label->setMaximumWidth(160);
  label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  StyleCompactSettingsField(label);
  label->setStyleSheet(
      style::sheet(QStringLiteral("time/value_field"), style::tokens()));
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

void TimePanel::refreshTimes() {
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

void TimePanel::syncAfterReset() {
  if (pause_button_ != nullptr && pause_button_->isChecked()) {
    pause_button_->blockSignals(true);
    pause_button_->setChecked(false);
    pause_button_->blockSignals(false);
  }
  refreshTimes();
}

void TimePanel::onResetClicked() {
  if (pause_button_ != nullptr && pause_button_->isChecked()) {
    pause_button_->setChecked(false);
  }
  // VisualizationFrame::onReset performs resetTime + viewport/Ogre cleanup.
  emit resetRequested();
}

void TimePanel::pauseToggled(bool checked) {
  if (manager_ != nullptr) {
    manager_->setTimePaused(checked);
  }
  if (pause_button_ != nullptr) {
    pause_button_->setText(checked ? tr("Resume") : tr("Pause"));
  }
}

void TimePanel::experimentalToggled(bool checked) {
  old_widget_->setVisible(!checked);
  experimental_widget_->setVisible(checked);
  // Keep Absolute Time visible in experimental mode via sim_time chip only when
  // legacy metrics are hidden — reparent a compact Time chip into experimental.
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
  notifyLayoutChanged();
}

void TimePanel::syncModeSelected(int mode) {
  if (manager_ != nullptr) {
    manager_->setTimeSyncMode(static_cast<common::TimeSyncMode>(mode));
  }
  sync_source_selector_->setEnabled(mode != 0);
  notifyLayoutChanged();
}

}  // namespace autoviz

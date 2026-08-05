/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/audio/audio_settings_widget.hpp"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace audio_panel {
namespace {

QStringList KnownChannels(common::VisualizationManager* manager) {
  QStringList channels;
  if (manager == nullptr) {
    return channels;
  }
  for (const integration::ChannelInfo& info : manager->channels()) {
    channels.push_back(QString::fromStdString(info.channel_name));
  }
  channels.sort(Qt::CaseInsensitive);
  return channels;
}

}  // namespace

AudioSettingsWidget::AudioSettingsWidget(common::VisualizationManager* manager,
                                         QWidget* parent)
    : manager_(manager), config_(DefaultAudioPanelConfig()), QWidget(parent) {
  ApplyCompactSettingsShell(this);
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                            PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  outer->setSpacing(PanelSettingsLayout::kOuterSpacing);
  outer->setAlignment(Qt::AlignTop);

  auto* general = new QGroupBox(tr("General"), this);
  StyleSettingsGroupBox(general);
  auto* form = new QFormLayout(general);
  ApplyCompactForm(form);
  title_edit_ = new QLineEdit(general);
  channel_combo_ = new QComboBox(general);
  channel_combo_->setEditable(true);
  color_button_ = MakeFlatActionButton(tr("Pick color"), general);
  volume_slider_ = new QSlider(Qt::Horizontal, general);
  volume_slider_->setRange(0, 100);
  mute_check_ = new QCheckBox(tr("Mute"), general);
  window_spin_ = new QDoubleSpinBox(general);
  window_spin_->setRange(1.0, 600.0);
  window_spin_->setDecimals(1);
  window_spin_->setSuffix(tr(" s"));

  form->addRow(tr("Title"), title_edit_);
  form->addRow(tr("Channel"), channel_combo_);
  form->addRow(tr("Color"), color_button_);
  form->addRow(tr("Volume"), volume_slider_);
  form->addRow(QString(), mute_check_);
  form->addRow(tr("Window size"), window_spin_);
  outer->addWidget(general);
  outer->addStretch();

  connect(title_edit_, &QLineEdit::textChanged, this,
          &AudioSettingsWidget::emitConfigChanged);
  connect(channel_combo_, &QComboBox::currentTextChanged, this,
          &AudioSettingsWidget::emitConfigChanged);
  connect(color_button_, &QPushButton::clicked, this, [this]() {
    const QColor chosen = QColorDialog::getColor(config_.waveform_color, this,
                                                 tr("Waveform color"));
    if (!chosen.isValid()) {
      return;
    }
    config_.waveform_color = chosen;
    UpdateColorButton(color_button_, chosen);
    emit configChanged();
  });
  connect(volume_slider_, &QSlider::valueChanged, this,
          &AudioSettingsWidget::emitConfigChanged);
  connect(mute_check_, &QCheckBox::toggled, this,
          &AudioSettingsWidget::emitConfigChanged);
  connect(window_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          &AudioSettingsWidget::emitConfigChanged);

  refreshChannels();
  applyToUi(config_);
}

AudioPanelConfig AudioSettingsWidget::config() const { return config_; }

void AudioSettingsWidget::setConfig(const AudioPanelConfig& config) {
  config_ = config;
  applyToUi(config_);
}

void AudioSettingsWidget::refreshChannels() {
  const QString current = config_.channel;
  const QStringList channels = KnownChannels(manager_);
  suppress_signals_ = true;
  channel_combo_->clear();
  for (const QString& channel : channels) {
    channel_combo_->addItem(channel);
  }
  if (!current.isEmpty() && channel_combo_->findText(current) < 0) {
    channel_combo_->addItem(current);
  }
  channel_combo_->setCurrentText(current);
  suppress_signals_ = false;
}

void AudioSettingsWidget::emitConfigChanged() {
  if (suppress_signals_) {
    return;
  }
  syncFromUi();
  emit configChanged();
}

void AudioSettingsWidget::applyToUi(const AudioPanelConfig& config) {
  suppress_signals_ = true;
  title_edit_->setText(config.title);
  channel_combo_->setCurrentText(config.channel);
  UpdateColorButton(color_button_, config.waveform_color);
  volume_slider_->setValue(static_cast<int>(config.volume * 100.0));
  mute_check_->setChecked(config.mute);
  window_spin_->setValue(config.window_size_sec);
  suppress_signals_ = false;
}

void AudioSettingsWidget::syncFromUi() {
  config_.title = title_edit_->text().trimmed();
  config_.channel = channel_combo_->currentText().trimmed();
  config_.volume = volume_slider_->value() / 100.0;
  config_.mute = mute_check_->isChecked();
  config_.window_size_sec = window_spin_->value();
}

}  // namespace audio_panel
}  // namespace autoviz

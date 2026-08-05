/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/gauge/gauge_settings_widget.hpp"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"

namespace autoviz {
namespace gauge {
namespace {

QWidget* MakeCollapsibleSection(QWidget* parent, const QString& title,
                                QWidget* body, bool expanded) {
  auto* section = new QGroupBox(title, parent);
  section->setCheckable(true);
  section->setChecked(expanded);
  auto* layout = new QVBoxLayout(section);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->addWidget(body);
  body->setVisible(expanded);
  QObject::connect(section, &QGroupBox::toggled, body, &QWidget::setVisible);
  return section;
}

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

GaugeSettingsWidget::GaugeSettingsWidget(common::VisualizationManager* manager,
                                         QWidget* parent)
    : manager_(manager), config_(DefaultGaugePanelConfig()), QWidget(parent) {
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(8, 8, 8, 8);
  outer->setSpacing(8);

  auto* title_row = new QHBoxLayout();
  title_row->addWidget(new QLabel(tr("Title"), this));
  title_edit_ = new QLineEdit(config_.title, this);
  title_row->addWidget(title_edit_, 1);
  outer->addLayout(title_row);

  auto* general_body = new QWidget(this);
  auto* general_form = new QFormLayout(general_body);
  channel_combo_ = new QComboBox(general_body);
  channel_combo_->setEditable(true);
  field_path_edit_ = new QLineEdit(config_.field_path, general_body);
  field_path_edit_->setPlaceholderText(tr("e.g. twist.linear.x"));
  min_spin_ = new QDoubleSpinBox(general_body);
  min_spin_->setRange(-1e12, 1e12);
  min_spin_->setDecimals(4);
  min_spin_->setValue(config_.min_value);
  max_spin_ = new QDoubleSpinBox(general_body);
  max_spin_->setRange(-1e12, 1e12);
  max_spin_->setDecimals(4);
  max_spin_->setValue(config_.max_value);
  general_form->addRow(tr("Channel"), channel_combo_);
  general_form->addRow(tr("Field path"), field_path_edit_);
  general_form->addRow(tr("Min"), min_spin_);
  general_form->addRow(tr("Max"), max_spin_);
  outer->addWidget(MakeCollapsibleSection(this, tr("General"), general_body, true));

  auto* appearance_body = new QWidget(this);
  auto* appearance_form = new QFormLayout(appearance_body);
  color_mode_combo_ = new QComboBox(appearance_body);
  color_mode_combo_->addItem(tr("Solid"), static_cast<int>(GaugeColorMode::kSolid));
  color_mode_combo_->addItem(tr("Gradient"), static_cast<int>(GaugeColorMode::kGradient));
  color_mode_combo_->addItem(tr("Color map"), static_cast<int>(GaugeColorMode::kColorMap));

  gradient_body_ = new QWidget(appearance_body);
  auto* gradient_layout = new QHBoxLayout(gradient_body_);
  gradient_layout->setContentsMargins(0, 0, 0, 0);
  gradient_start_button_ = new QPushButton(tr("Start"), gradient_body_);
  gradient_end_button_ = new QPushButton(tr("End"), gradient_body_);
  gradient_layout->addWidget(gradient_start_button_);
  gradient_layout->addWidget(gradient_end_button_);

  color_map_body_ = new QWidget(appearance_body);
  auto* color_map_layout = new QVBoxLayout(color_map_body_);
  color_map_layout->setContentsMargins(0, 0, 0, 0);
  color_map_combo_ = new QComboBox(color_map_body_);
  color_map_combo_->addItem(tr("Red → Yellow → Green"),
                            static_cast<int>(GaugeColorMap::kRedYellowGreen));
  color_map_combo_->addItem(tr("Rainbow"), static_cast<int>(GaugeColorMap::kRainbow));
  color_map_combo_->addItem(tr("Turbo"), static_cast<int>(GaugeColorMap::kTurbo));
  color_map_layout->addWidget(color_map_combo_);

  reverse_color_check_ = new QCheckBox(tr("Reverse color"), appearance_body);
  reverse_direction_check_ = new QCheckBox(tr("Reverse direction"), appearance_body);

  appearance_form->addRow(tr("Color mode"), color_mode_combo_);
  appearance_form->addRow(tr("Gradient"), gradient_body_);
  appearance_form->addRow(tr("Color map"), color_map_body_);
  appearance_form->addRow(QString(), reverse_color_check_);
  appearance_form->addRow(QString(), reverse_direction_check_);
  outer->addWidget(
      MakeCollapsibleSection(this, tr("Appearance"), appearance_body, true));
  outer->addStretch();

  connect(title_edit_, &QLineEdit::textChanged, this,
          &GaugeSettingsWidget::emitConfigChanged);
  connect(channel_combo_, &QComboBox::currentTextChanged, this,
          &GaugeSettingsWidget::emitConfigChanged);
  connect(field_path_edit_, &QLineEdit::textChanged, this,
          &GaugeSettingsWidget::emitConfigChanged);
  connect(min_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &GaugeSettingsWidget::emitConfigChanged);
  connect(max_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &GaugeSettingsWidget::emitConfigChanged);
  connect(color_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this]() {
            syncColorModeUi();
            emitConfigChanged();
          });
  connect(color_map_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &GaugeSettingsWidget::emitConfigChanged);
  connect(gradient_start_button_, &QPushButton::clicked, this,
          [this]() { pickGradientColor(true); });
  connect(gradient_end_button_, &QPushButton::clicked, this,
          [this]() { pickGradientColor(false); });
  connect(reverse_color_check_, &QCheckBox::toggled, this,
          &GaugeSettingsWidget::emitConfigChanged);
  connect(reverse_direction_check_, &QCheckBox::toggled, this,
          &GaugeSettingsWidget::emitConfigChanged);

  refreshChannels();
  setConfig(config_);
}

GaugePanelConfig GaugeSettingsWidget::config() const {
  GaugePanelConfig out = config_;
  out.title = title_edit_->text().trimmed();
  out.channel = channel_combo_->currentText().trimmed();
  out.field_path = field_path_edit_->text().trimmed();
  out.min_value = min_spin_->value();
  out.max_value = max_spin_->value();
  out.color_mode = static_cast<GaugeColorMode>(color_mode_combo_->currentData().toInt());
  out.color_map = static_cast<GaugeColorMap>(color_map_combo_->currentData().toInt());
  out.reverse_color = reverse_color_check_->isChecked();
  out.reverse_direction = reverse_direction_check_->isChecked();
  return out;
}

void GaugeSettingsWidget::setConfig(const GaugePanelConfig& config) {
  config_ = config;
  title_edit_->setText(config_.title);
  field_path_edit_->setText(config_.field_path);
  min_spin_->setValue(config_.min_value);
  max_spin_->setValue(config_.max_value);
  color_mode_combo_->setCurrentIndex(
      color_mode_combo_->findData(static_cast<int>(config_.color_mode)));
  color_map_combo_->setCurrentIndex(
      color_map_combo_->findData(static_cast<int>(config_.color_map)));
  reverse_color_check_->setChecked(config_.reverse_color);
  reverse_direction_check_->setChecked(config_.reverse_direction);
  gradient_start_button_->setStyleSheet(
      QStringLiteral("background: %1;").arg(config_.gradient_start.name()));
  gradient_end_button_->setStyleSheet(
      QStringLiteral("background: %1;").arg(config_.gradient_end.name()));

  const int channel_index = channel_combo_->findText(config_.channel);
  if (channel_index >= 0) {
    channel_combo_->setCurrentIndex(channel_index);
  } else {
    channel_combo_->setEditText(config_.channel);
  }
  syncColorModeUi();
}

void GaugeSettingsWidget::refreshChannels() {
  const QString current = channel_combo_->currentText();
  channel_combo_->blockSignals(true);
  channel_combo_->clear();
  channel_combo_->addItems(KnownChannels(manager_));
  const int index = channel_combo_->findText(current);
  if (index >= 0) {
    channel_combo_->setCurrentIndex(index);
  } else if (!current.isEmpty()) {
    channel_combo_->setEditText(current);
  }
  channel_combo_->blockSignals(false);
}

void GaugeSettingsWidget::syncColorModeUi() {
  const auto mode = static_cast<GaugeColorMode>(color_mode_combo_->currentData().toInt());
  gradient_body_->setVisible(mode == GaugeColorMode::kGradient ||
                             mode == GaugeColorMode::kSolid);
  color_map_body_->setVisible(mode == GaugeColorMode::kColorMap);
  gradient_start_button_->setVisible(mode == GaugeColorMode::kGradient);
}

void GaugeSettingsWidget::pickGradientColor(bool start) {
  const QColor initial = start ? config_.gradient_start : config_.gradient_end;
  const QColor chosen =
      QColorDialog::getColor(initial, this, start ? tr("Gradient start")
                                                  : tr("Gradient end"));
  if (!chosen.isValid()) {
    return;
  }
  if (start) {
    config_.gradient_start = chosen;
    gradient_start_button_->setStyleSheet(
        QStringLiteral("background: %1;").arg(chosen.name()));
  } else {
    config_.gradient_end = chosen;
    gradient_end_button_->setStyleSheet(
        QStringLiteral("background: %1;").arg(chosen.name()));
  }
  emitConfigChanged();
}

void GaugeSettingsWidget::emitConfigChanged() { emit configChanged(); }

}  // namespace gauge
}  // namespace autoviz

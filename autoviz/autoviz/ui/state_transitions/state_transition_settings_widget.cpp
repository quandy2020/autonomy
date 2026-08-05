/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/state_transitions/state_transition_settings_widget.hpp"

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
#include <QScrollArea>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace state_transitions {
namespace {

QStringList AllChannels(common::VisualizationManager* manager) {
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

StateTransitionSettingsWidget::StateTransitionSettingsWidget(
    common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultStateTransitionPanelConfig()), QWidget(parent) {
  ApplyCompactSettingsShell(this);
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                            PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  outer->setSpacing(PanelSettingsLayout::kOuterSpacing);
  outer->setAlignment(Qt::AlignTop);

  auto* title_form = new QFormLayout();
  ApplyCompactForm(title_form);
  title_edit_ = new QLineEdit(config_.title, this);
  title_form->addRow(tr("Title"), title_edit_);
  outer->addLayout(title_form);

  auto* axis_group = new QGroupBox(tr("X-axis"), this);
  StyleSettingsGroupBox(axis_group);
  auto* axis_form = new QFormLayout(axis_group);
  ApplyCompactForm(axis_form);
  x_axis_mode_combo_ = new QComboBox(axis_group);
  x_axis_mode_combo_->addItem(tr("Sliding window"), static_cast<int>(StateXAxisMode::kSlidingWindow));
  x_axis_mode_combo_->addItem(tr("Fixed range"), static_cast<int>(StateXAxisMode::kFixedRange));
  window_spin_ = new QDoubleSpinBox(axis_group);
  window_spin_->setRange(1.0, 86400.0);
  window_spin_->setDecimals(1);
  window_spin_->setSuffix(tr(" s"));
  window_spin_->setValue(config_.x_window_sec);
  fixed_min_spin_ = new QDoubleSpinBox(axis_group);
  fixed_max_spin_ = new QDoubleSpinBox(axis_group);
  fixed_min_spin_->setRange(-1e9, 1e9);
  fixed_max_spin_->setRange(-1e9, 1e9);
  fixed_min_spin_->setDecimals(3);
  fixed_max_spin_->setDecimals(3);
  axis_form->addRow(tr("Mode"), x_axis_mode_combo_);
  axis_form->addRow(tr("Window"), window_spin_);
  axis_form->addRow(tr("Fixed min"), fixed_min_spin_);
  axis_form->addRow(tr("Fixed max"), fixed_max_spin_);
  outer->addWidget(axis_group);

  auto* series_group = new QGroupBox(tr("Series"), this);
  StyleSettingsGroupBox(series_group);
  auto* series_group_layout = new QVBoxLayout(series_group);
  series_group_layout->setContentsMargins(6, 4, 6, 6);
  series_group_layout->setSpacing(4);
  series_group_layout->setAlignment(Qt::AlignTop);
  series_container_ = new QWidget(series_group);
  series_layout_ = new QVBoxLayout(series_container_);
  series_layout_->setContentsMargins(0, 0, 0, 0);
  series_layout_->setSpacing(4);
  series_layout_->setAlignment(Qt::AlignTop);
  series_group_layout->addWidget(series_container_);
  auto* add_series = MakeFlatActionButton(tr("+ Add series"), series_group);
  series_group_layout->addWidget(add_series, 0, Qt::AlignLeft);
  outer->addWidget(series_group, 1);

  connect(title_edit_, &QLineEdit::textChanged, this,
          &StateTransitionSettingsWidget::emitConfigChanged);
  connect(x_axis_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this]() {
            updateAxisFieldVisibility();
            emitConfigChanged();
          });
  connect(window_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &StateTransitionSettingsWidget::emitConfigChanged);
  connect(fixed_min_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &StateTransitionSettingsWidget::emitConfigChanged);
  connect(fixed_max_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &StateTransitionSettingsWidget::emitConfigChanged);
  connect(add_series, &QPushButton::clicked, this,
          &StateTransitionSettingsWidget::addSeriesRequested);

  setConfig(config_);
}

StateTransitionPanelConfig StateTransitionSettingsWidget::config() const {
  StateTransitionPanelConfig out = config_;
  out.title = title_edit_->text().trimmed();
  out.x_axis_mode = static_cast<StateXAxisMode>(
      x_axis_mode_combo_->currentData().toInt());
  out.x_window_sec = window_spin_->value();
  out.fixed_min_time = fixed_min_spin_->value();
  out.fixed_max_time = fixed_max_spin_->value();
  return out;
}

void StateTransitionSettingsWidget::setConfig(
    const StateTransitionPanelConfig& config) {
  syncing_ = true;
  config_ = config;
  title_edit_->setText(config_.title);
  x_axis_mode_combo_->setCurrentIndex(
      config_.x_axis_mode == StateXAxisMode::kFixedRange ? 1 : 0);
  window_spin_->setValue(config_.x_window_sec);
  fixed_min_spin_->setValue(config_.fixed_min_time);
  fixed_max_spin_->setValue(config_.fixed_max_time);
  updateAxisFieldVisibility();
  rebuildSeriesEditors();
  syncing_ = false;
}

void StateTransitionSettingsWidget::updateAxisFieldVisibility() {
  if (x_axis_mode_combo_ == nullptr || window_spin_ == nullptr ||
      fixed_min_spin_ == nullptr || fixed_max_spin_ == nullptr) {
    return;
  }
  const bool sliding =
      static_cast<StateXAxisMode>(x_axis_mode_combo_->currentData().toInt()) ==
      StateXAxisMode::kSlidingWindow;
  window_spin_->setVisible(sliding);
  fixed_min_spin_->setVisible(!sliding);
  fixed_max_spin_->setVisible(!sliding);
  if (auto* axis_group = qobject_cast<QGroupBox*>(x_axis_mode_combo_->parent())) {
    if (QFormLayout* form = qobject_cast<QFormLayout*>(axis_group->layout())) {
      if (QLabel* label = qobject_cast<QLabel*>(form->labelForField(window_spin_))) {
        label->setVisible(sliding);
      }
      if (QLabel* label = qobject_cast<QLabel*>(form->labelForField(fixed_min_spin_))) {
        label->setVisible(!sliding);
      }
      if (QLabel* label = qobject_cast<QLabel*>(form->labelForField(fixed_max_spin_))) {
        label->setVisible(!sliding);
      }
    }
  }
}

void StateTransitionSettingsWidget::refreshChannels() {
  rebuildSeriesEditors();
}

QWidget* StateTransitionSettingsWidget::createSeriesEditor(int index) {
  if (index < 0 || index >= config_.series.size()) {
    return new QWidget(this);
  }
  StateTransitionSeriesConfig& series = config_.series[index];
  auto* box = new QGroupBox(
      series.label.isEmpty() ? tr("Series %1").arg(index + 1) : series.label,
      series_container_);
  StyleSettingsGroupBox(box);
  auto* form = new QFormLayout(box);
  ApplyCompactForm(form);

  auto* channel_combo = new QComboBox(box);
  channel_combo->setEditable(true);
  channel_combo->addItems(AllChannels(manager_));
  const int channel_index = channel_combo->findText(series.channel);
  if (channel_index >= 0) {
    channel_combo->setCurrentIndex(channel_index);
  } else {
    channel_combo->setEditText(series.channel);
  }

  auto* field_path = new QLineEdit(series.field_path, box);
  field_path->setPlaceholderText(tr("e.g. mode or objects[:]{id==$id}.state"));
  auto* label_edit = new QLineEdit(series.label, box);
  auto* enabled_check = new QCheckBox(tr("Enabled"), box);
  enabled_check->setChecked(series.enabled);
  auto* timestamp_combo = new QComboBox(box);
  timestamp_combo->addItem(tr("Log time"),
                           static_cast<int>(plot::PlotTimestampMode::kLogTime));
  timestamp_combo->addItem(tr("Receive time"),
                           static_cast<int>(plot::PlotTimestampMode::kReceiveTime));
  timestamp_combo->addItem(tr("Custom field"),
                           static_cast<int>(plot::PlotTimestampMode::kCustomField));
  timestamp_combo->setCurrentIndex(static_cast<int>(series.timestamp_mode));
  auto* custom_ts = new QLineEdit(series.custom_timestamp_path, box);
  custom_ts->setPlaceholderText(tr("header.stamp"));

  form->addRow(tr("Channel"), channel_combo);
  form->addRow(tr("Message path"), field_path);
  form->addRow(tr("Label"), label_edit);
  form->addRow(QString(), enabled_check);
  form->addRow(tr("Timestamp"), timestamp_combo);
  form->addRow(tr("Custom timestamp"), custom_ts);
  custom_ts->setEnabled(series.timestamp_mode ==
                        plot::PlotTimestampMode::kCustomField);

  auto* customize_group = new QGroupBox(tr("Customize"), box);
  StyleSettingsGroupBox(customize_group);
  auto* customize_layout = new QVBoxLayout(customize_group);
  customize_layout->setContentsMargins(6, 4, 6, 6);
  customize_layout->setSpacing(4);
  customize_layout->setAlignment(Qt::AlignTop);
  for (int mapping_index = 0; mapping_index < series.mappings.size();
       ++mapping_index) {
    customize_layout->addWidget(
        createMappingRuleEditor(index, mapping_index));
  }
  auto* add_mapping = MakeFlatActionButton(tr("+ Add mapping"), customize_group);
  customize_layout->addWidget(add_mapping, 0, Qt::AlignLeft);
  form->addRow(customize_group);

  auto* remove_button = MakeDestructiveFlatActionButton(tr("Remove series"), box);
  form->addRow(remove_button);

  auto wire = [this, index]() {
    if (syncing_) {
      return;
    }
    emitConfigChanged();
  };
  connect(channel_combo, &QComboBox::currentTextChanged, this, wire);
  connect(field_path, &QLineEdit::textEdited, this, wire);
  connect(label_edit, &QLineEdit::textEdited, this, wire);
  connect(enabled_check, &QCheckBox::toggled, this, wire);
  connect(timestamp_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          wire);
  connect(custom_ts, &QLineEdit::textEdited, this, wire);
  connect(remove_button, &QPushButton::clicked, this, [this, index]() {
    emit removeSeriesRequested(index);
  });
  connect(add_mapping, &QPushButton::clicked, this, [this, index]() {
    if (index < 0 || index >= config_.series.size()) {
      return;
    }
    config_.series[index].mappings.push_back(StateMappingRule{});
    rebuildSeriesEditors();
    emitConfigChanged();
  });

  connect(channel_combo, &QComboBox::currentTextChanged, this,
          [this, index, channel_combo](const QString& text) {
            if (syncing_ || index >= config_.series.size()) {
              return;
            }
            config_.series[index].channel = text.trimmed();
          });
  connect(field_path, &QLineEdit::textEdited, this,
          [this, index, field_path](const QString& text) {
            if (index >= config_.series.size()) {
              return;
            }
            config_.series[index].field_path = text.trimmed();
          });
  connect(label_edit, &QLineEdit::textEdited, this,
          [this, index, label_edit](const QString& text) {
            if (index >= config_.series.size()) {
              return;
            }
            config_.series[index].label = text.trimmed();
          });
  connect(enabled_check, &QCheckBox::toggled, this, [this, index](bool enabled) {
    if (index >= config_.series.size()) {
      return;
    }
    config_.series[index].enabled = enabled;
  });
  connect(timestamp_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this, index, timestamp_combo, custom_ts](int combo_index) {
            if (index >= config_.series.size()) {
              return;
            }
            config_.series[index].timestamp_mode =
                static_cast<plot::PlotTimestampMode>(
                    timestamp_combo->itemData(combo_index).toInt());
            custom_ts->setEnabled(config_.series[index].timestamp_mode ==
                                  plot::PlotTimestampMode::kCustomField);
          });
  connect(custom_ts, &QLineEdit::textEdited, this,
          [this, index, custom_ts](const QString& text) {
            if (index >= config_.series.size()) {
              return;
            }
            config_.series[index].custom_timestamp_path = text.trimmed();
          });
  return box;
}

QWidget* StateTransitionSettingsWidget::createMappingRuleEditor(
    int series_index, int mapping_index) {
  if (series_index < 0 || series_index >= config_.series.size() || mapping_index < 0 ||
      mapping_index >= config_.series[series_index].mappings.size()) {
    return new QWidget(this);
  }
  const StateMappingRule& rule =
      config_.series[series_index].mappings[mapping_index];
  auto* row = new QGroupBox(
      rule.kind == StateMappingKind::kExact ? tr("Exact match") : tr("Numeric range"),
      series_container_);
  StyleSettingsGroupBox(row);
  auto* form = new QFormLayout(row);
  ApplyCompactForm(form);

  auto* kind_combo = new QComboBox(row);
  kind_combo->addItem(tr("Exact match"), static_cast<int>(StateMappingKind::kExact));
  kind_combo->addItem(tr("Numeric range"),
                      static_cast<int>(StateMappingKind::kNumericRange));
  kind_combo->setCurrentIndex(rule.kind == StateMappingKind::kNumericRange ? 1 : 0);

  auto* match_value = new QLineEdit(rule.match_value, row);
  match_value->setPlaceholderText(tr("Value to match"));
  auto* range_min = new QDoubleSpinBox(row);
  auto* range_max = new QDoubleSpinBox(row);
  range_min->setRange(-1e12, 1e12);
  range_max->setRange(-1e12, 1e12);
  range_min->setDecimals(6);
  range_max->setDecimals(6);
  range_min->setValue(rule.range_min);
  range_max->setValue(rule.range_max);
  auto* label_edit = new QLineEdit(rule.label, row);
  auto* color_button = new QPushButton(row);
  UpdateColorButton(color_button, rule.color);

  form->addRow(tr("Type"), kind_combo);
  form->addRow(tr("Match value"), match_value);
  form->addRow(tr("Range min"), range_min);
  form->addRow(tr("Range max"), range_max);
  form->addRow(tr("Label"), label_edit);
  form->addRow(tr("Color"), color_button);

  const bool is_exact = rule.kind == StateMappingKind::kExact;
  match_value->setVisible(is_exact);
  range_min->setVisible(!is_exact);
  range_max->setVisible(!is_exact);
  if (QLabel* label = qobject_cast<QLabel*>(form->labelForField(match_value))) {
    label->setVisible(is_exact);
  }
  if (QLabel* label = qobject_cast<QLabel*>(form->labelForField(range_min))) {
    label->setVisible(!is_exact);
  }
  if (QLabel* label = qobject_cast<QLabel*>(form->labelForField(range_max))) {
    label->setVisible(!is_exact);
  }

  auto* remove_button = MakeDestructiveFlatActionButton(tr("Remove mapping"), row);
  form->addRow(remove_button);

  auto apply_kind_visibility = [this, row, form, match_value, range_min, range_max](
                                   StateMappingKind kind) {
    const bool exact = kind == StateMappingKind::kExact;
    match_value->setVisible(exact);
    range_min->setVisible(!exact);
    range_max->setVisible(!exact);
    if (QLabel* label = qobject_cast<QLabel*>(form->labelForField(match_value))) {
      label->setVisible(exact);
    }
    if (QLabel* label = qobject_cast<QLabel*>(form->labelForField(range_min))) {
      label->setVisible(!exact);
    }
    if (QLabel* label = qobject_cast<QLabel*>(form->labelForField(range_max))) {
      label->setVisible(!exact);
    }
    row->setTitle(exact ? tr("Exact match") : tr("Numeric range"));
  };

  auto sync_rule = [this, series_index, mapping_index, kind_combo, match_value,
                    range_min, range_max, label_edit]() {
    if (syncing_ || series_index < 0 || series_index >= config_.series.size() ||
        mapping_index < 0 ||
        mapping_index >= config_.series[series_index].mappings.size()) {
      return;
    }
    StateMappingRule& mapping = config_.series[series_index].mappings[mapping_index];
    mapping.kind = static_cast<StateMappingKind>(kind_combo->currentData().toInt());
    mapping.match_value = match_value->text().trimmed();
    mapping.range_min = range_min->value();
    mapping.range_max = range_max->value();
    mapping.label = label_edit->text().trimmed();
    emitConfigChanged();
  };

  connect(kind_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [apply_kind_visibility, kind_combo, sync_rule](int) {
            apply_kind_visibility(static_cast<StateMappingKind>(
                kind_combo->currentData().toInt()));
            sync_rule();
          });
  connect(match_value, &QLineEdit::textEdited, this,
          [sync_rule](const QString&) { sync_rule(); });
  connect(range_min, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [sync_rule](double) { sync_rule(); });
  connect(range_max, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [sync_rule](double) { sync_rule(); });
  connect(label_edit, &QLineEdit::textEdited, this,
          [sync_rule](const QString&) { sync_rule(); });
  connect(color_button, &QPushButton::clicked, this,
          [this, series_index, mapping_index, color_button]() {
            if (series_index < 0 || series_index >= config_.series.size() ||
                mapping_index < 0 ||
                mapping_index >= config_.series[series_index].mappings.size()) {
              return;
            }
            const QColor chosen = QColorDialog::getColor(
                config_.series[series_index].mappings[mapping_index].color, this);
            if (!chosen.isValid()) {
              return;
            }
            config_.series[series_index].mappings[mapping_index].color = chosen;
            UpdateColorButton(color_button, chosen);
            emitConfigChanged();
          });
  connect(remove_button, &QPushButton::clicked, this, [this, series_index, mapping_index]() {
    if (series_index < 0 || series_index >= config_.series.size() || mapping_index < 0 ||
        mapping_index >= config_.series[series_index].mappings.size()) {
      return;
    }
    config_.series[series_index].mappings.removeAt(mapping_index);
    rebuildSeriesEditors();
    emitConfigChanged();
  });
  return row;
}

void StateTransitionSettingsWidget::rebuildSeriesEditors() {
  if (series_layout_ == nullptr) {
    return;
  }
  syncing_ = true;
  while (QLayoutItem* item = series_layout_->takeAt(0)) {
    if (item->widget() != nullptr) {
      item->widget()->deleteLater();
    }
    delete item;
  }
  for (int i = 0; i < config_.series.size(); ++i) {
    series_layout_->addWidget(createSeriesEditor(i));
  }
  series_layout_->addStretch();
  syncing_ = false;
}

void StateTransitionSettingsWidget::emitConfigChanged() {
  if (syncing_) {
    return;
  }
  config_.title = title_edit_->text().trimmed();
  config_.x_axis_mode = static_cast<StateXAxisMode>(
      x_axis_mode_combo_->currentData().toInt());
  config_.x_window_sec = window_spin_->value();
  config_.fixed_min_time = fixed_min_spin_->value();
  config_.fixed_max_time = fixed_max_spin_->value();
  emit configChanged();
}

}  // namespace state_transitions
}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_settings_widget.hpp"

#include <functional>

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
#include "autoviz/integration/channel_manager.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace plot {
namespace {

QWidget* MakeSegmentedToggle(QWidget* parent, const QStringList& labels,
                             int checked_index,
                             const std::function<void(int)>& on_changed) {
  auto* container = new QWidget(parent);
  auto* layout = new QHBoxLayout(container);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  container->setStyleSheet(SegmentedToggleStyle());
  for (int i = 0; i < labels.size(); ++i) {
    auto* button = new QPushButton(labels.at(i), container);
    button->setCheckable(true);
    button->setChecked(i == checked_index);
    button->setAutoExclusive(true);
    layout->addWidget(button);
    QObject::connect(button, &QPushButton::clicked, container, [on_changed, i]() {
      on_changed(i);
    });
  }
  return container;
}

}  // namespace

PlotSettingsWidget::PlotSettingsWidget(common::VisualizationManager* manager,
                                       QWidget* parent)
    : manager_(manager), QWidget(parent) {
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
  connect(title_edit_, &QLineEdit::textChanged, this,
          &PlotSettingsWidget::emitConfigChanged);

  auto* general_body = new QWidget(this);
  auto* general_form = new QFormLayout(general_body);
  ApplyCompactForm(general_form);
  x_axis_mode_combo_ = new QComboBox(general_body);
  x_axis_mode_combo_->addItem(tr("Timestamp"),
                              static_cast<int>(PlotXAxisMode::kTimestamp));
  x_axis_mode_combo_->addItem(tr("Index"),
                              static_cast<int>(PlotXAxisMode::kIndex));
  x_axis_mode_combo_->addItem(tr("Message path"),
                              static_cast<int>(PlotXAxisMode::kMessagePath));
  general_form->addRow(tr("X-axis value type"), x_axis_mode_combo_);
  message_path_mode_combo_ = new QComboBox(general_body);
  message_path_mode_combo_->addItem(tr("Current"),
                                    static_cast<int>(PlotMessagePathMode::kCurrent));
  message_path_mode_combo_->addItem(tr("Accumulated"),
                                    static_cast<int>(PlotMessagePathMode::kAccumulated));
  message_path_mode_combo_->setCurrentIndex(
      config_.message_path_mode == PlotMessagePathMode::kAccumulated ? 1 : 0);
  general_form->addRow(tr("Message path mode"), message_path_mode_combo_);
  sync_plots_combo_ = new QComboBox(general_body);
  sync_plots_combo_->addItem(tr("Off"), false);
  sync_plots_combo_->addItem(tr("On"), true);
  sync_plots_combo_->setCurrentIndex(config_.sync_with_other_plots ? 1 : 0);
  general_form->addRow(tr("Sync x-axis with other plots"), sync_plots_combo_);
  connect(sync_plots_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &PlotSettingsWidget::emitConfigChanged);
  connect(message_path_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &PlotSettingsWidget::emitConfigChanged);
  outer->addWidget(makeCollapsibleSection(tr("General"), general_body, true));

  auto* legend_body = new QWidget(this);
  auto* legend_layout = new QVBoxLayout(legend_body);
  legend_layout->setContentsMargins(0, 0, 0, 0);
  show_legend_values_check_ = new QCheckBox(tr("Show values"), legend_body);
  show_legend_values_check_->setChecked(config_.show_legend_values);
  legend_layout->addWidget(show_legend_values_check_);
  connect(show_legend_values_check_, &QCheckBox::toggled, this,
          &PlotSettingsWidget::emitConfigChanged);
  outer->addWidget(makeCollapsibleSection(tr("Legend"), legend_body, true));

  x_axis_timestamp_body_ = new QWidget(this);
  auto* timestamp_form = new QFormLayout(x_axis_timestamp_body_);
  x_window_spin_ = new QDoubleSpinBox(x_axis_timestamp_body_);
  x_window_spin_->setRange(1.0, 3600.0);
  x_window_spin_->setDecimals(1);
  x_window_spin_->setSuffix(tr(" s"));
  x_window_spin_->setValue(config_.x_window_sec);
  timestamp_form->addRow(tr("Visible time range"), x_window_spin_);
  timestamp_form->addRow(
      new QLabel(tr("Timestamp source is configured per series (Log time / "
                    "Receive time)."),
                 x_axis_timestamp_body_));
  connect(x_window_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &PlotSettingsWidget::emitConfigChanged);

  x_axis_index_body_ = new QWidget(this);
  auto* index_layout = new QVBoxLayout(x_axis_index_body_);
  index_layout->setContentsMargins(0, 0, 0, 0);
  index_layout->addWidget(new QLabel(
      tr("X axis uses array index (0, 1, 2, …). Plot the Y-value path of an "
         "array field in each series."),
      x_axis_index_body_));
  x_axis_index_body_->setVisible(false);

  x_axis_message_path_body_ = new QWidget(this);
  auto* message_path_layout = new QVBoxLayout(x_axis_message_path_body_);
  message_path_layout->setContentsMargins(0, 0, 0, 0);
  message_path_layout->addWidget(new QLabel(
      tr("X and Y values come from message fields configured per series. "
         "Use Current to show only the latest point, or Accumulated for history."),
      x_axis_message_path_body_));
  x_axis_message_path_body_->setVisible(false);

  auto* x_axis_body = new QWidget(this);
  auto* x_axis_layout = new QVBoxLayout(x_axis_body);
  x_axis_layout->setContentsMargins(0, 0, 0, 0);
  x_axis_layout->addWidget(x_axis_timestamp_body_);
  x_axis_layout->addWidget(x_axis_index_body_);
  x_axis_layout->addWidget(x_axis_message_path_body_);
  outer->addWidget(makeCollapsibleSection(tr("X-axis"), x_axis_body, false));
  connect(x_axis_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int /*index*/) {
            updateAxisModeVisibility();
            emitConfigChanged();
          });

  auto* y_axis_body = new QWidget(this);
  auto* y_axis_layout = new QVBoxLayout(y_axis_body);
  y_axis_layout->setContentsMargins(0, 0, 0, 0);
  lock_axis_scales_check_ = new QCheckBox(tr("Locked axis scales (1:1)"), y_axis_body);
  lock_axis_scales_check_->setChecked(config_.lock_axis_scales);
  y_axis_layout->addWidget(lock_axis_scales_check_);
  connect(lock_axis_scales_check_, &QCheckBox::toggled, this,
          &PlotSettingsWidget::emitConfigChanged);
  outer->addWidget(makeCollapsibleSection(tr("Y-axis"), y_axis_body, false));

  updateAxisModeVisibility();

  series_container_ = new QWidget(this);
  series_list_layout_ = new QVBoxLayout(series_container_);
  series_list_layout_->setContentsMargins(0, 0, 0, 0);
  series_list_layout_->setSpacing(8);

  auto* series_header = new QHBoxLayout();
  series_header->addWidget(new QLabel(tr("Series"), this));
  series_header->addStretch();
  add_series_button_ = MakeFlatActionButton(QStringLiteral("+ Add series"), this);
  connect(add_series_button_, &QPushButton::clicked, this,
          &PlotSettingsWidget::onAddSeriesClicked);
  series_header->addWidget(add_series_button_);
  auto* series_body = new QWidget(this);
  auto* series_body_layout = new QVBoxLayout(series_body);
  series_body_layout->setContentsMargins(0, 0, 0, 0);
  series_body_layout->addLayout(series_header);
  series_body_layout->addWidget(series_container_);
  outer->addWidget(makeCollapsibleSection(tr("Series"), series_body, true));

  outer->addStretch();
  rebuildSeriesSection();
}

QWidget* PlotSettingsWidget::makeCollapsibleSection(const QString& title,
                                                    QWidget* body, bool expanded) {
  return MakeCollapsibleSection(this, title, body, expanded);
}

void PlotSettingsWidget::updateAxisModeVisibility() {
  const auto mode =
      static_cast<PlotXAxisMode>(x_axis_mode_combo_->currentData().toInt());
  if (x_axis_timestamp_body_ != nullptr) {
    x_axis_timestamp_body_->setVisible(mode == PlotXAxisMode::kTimestamp);
  }
  if (x_axis_index_body_ != nullptr) {
    x_axis_index_body_->setVisible(mode == PlotXAxisMode::kIndex);
  }
  if (x_axis_message_path_body_ != nullptr) {
    x_axis_message_path_body_->setVisible(mode == PlotXAxisMode::kMessagePath);
  }
  if (message_path_mode_combo_ != nullptr) {
    message_path_mode_combo_->setEnabled(mode == PlotXAxisMode::kMessagePath);
  }
}

void PlotSettingsWidget::setConfig(const PlotPanelConfig& config) {
  config_ = config;
  title_edit_->setText(config_.title);
  const int mode_index = x_axis_mode_combo_->findData(static_cast<int>(config_.x_axis_mode));
  if (mode_index >= 0) {
    x_axis_mode_combo_->setCurrentIndex(mode_index);
  }
  if (message_path_mode_combo_ != nullptr) {
    message_path_mode_combo_->setCurrentIndex(
        config_.message_path_mode == PlotMessagePathMode::kAccumulated ? 1 : 0);
  }
  sync_plots_combo_->setCurrentIndex(config_.sync_with_other_plots ? 1 : 0);
  if (x_window_spin_ != nullptr) {
    x_window_spin_->setValue(config_.x_window_sec);
  }
  if (lock_axis_scales_check_ != nullptr) {
    lock_axis_scales_check_->setChecked(config_.lock_axis_scales);
  }
  updateAxisModeVisibility();
  if (show_legend_values_check_ != nullptr) {
    show_legend_values_check_->setChecked(config_.show_legend_values);
  }
  rebuildSeriesSection();
}

PlotPanelConfig PlotSettingsWidget::config() const { return config_; }

QString PlotSettingsWidget::defaultSeriesLabel(int index) {
  return QObject::tr("Series %1").arg(index + 1);
}

void PlotSettingsWidget::refreshChannelLists() { rebuildSeriesSection(); }

QWidget* PlotSettingsWidget::buildSeriesEditor(int index,
                                               const PlotSeriesConfig& series) {
  auto* box = new QGroupBox(defaultSeriesLabel(index), series_container_);
  StyleSettingsGroupBox(box);
  auto* form = new QFormLayout(box);
  ApplyCompactForm(form);

  auto* channel_combo = new QComboBox(box);
  channel_combo->setEditable(true);
  if (manager_ != nullptr) {
    for (const integration::ChannelInfo& channel : manager_->channels()) {
      channel_combo->addItem(QString::fromStdString(channel.channel_name));
    }
  }
  channel_combo->setCurrentText(series.channel);
  form->addRow(tr("Channel"), channel_combo);

  auto* field_edit = new QLineEdit(series.field_path, box);
  field_edit->setPlaceholderText(tr("e.g. pose.pose.position.x or .@abs"));
  form->addRow(tr("Y-value path"), field_edit);

  auto* x_field_edit = new QLineEdit(series.x_field_path, box);
  x_field_edit->setPlaceholderText(tr("X-value path (message path mode)"));
  form->addRow(tr("X-value path"), x_field_edit);

  auto* label_edit = new QLineEdit(series.label, box);
  form->addRow(tr("Label"), label_edit);

  auto* color_button = new QPushButton(box);
  UpdateColorButton(color_button, series.color);
  form->addRow(tr("Color"), color_button);

  auto* line_size_edit = new QLineEdit(series.line_size, box);
  form->addRow(tr("Line size"), line_size_edit);

  auto* show_line = MakeSegmentedToggle(
      box, {tr("Off"), tr("On")}, series.show_line ? 1 : 0,
      [this, index](int selected) {
        if (index >= 0 && index < config_.series.size()) {
          config_.series[index].show_line = selected == 1;
          emit configChanged();
        }
      });
  form->addRow(tr("Show line"), show_line);

  auto* timestamp_combo = new QComboBox(box);
  timestamp_combo->addItem(tr("Log time"),
                           static_cast<int>(PlotTimestampMode::kLogTime));
  timestamp_combo->addItem(tr("Receive time"),
                           static_cast<int>(PlotTimestampMode::kReceiveTime));
  timestamp_combo->addItem(tr("Custom field"),
                           static_cast<int>(PlotTimestampMode::kCustomField));
  const int ts_index = timestamp_combo->findData(static_cast<int>(series.timestamp_mode));
  timestamp_combo->setCurrentIndex(ts_index >= 0 ? ts_index : 0);
  form->addRow(tr("Timestamp"), timestamp_combo);

  auto* custom_ts_edit = new QLineEdit(series.custom_timestamp_path, box);
  custom_ts_edit->setPlaceholderText(tr("e.g. header.stamp"));
  form->addRow(tr("Custom timestamp path"), custom_ts_edit);
  custom_ts_edit->setEnabled(series.timestamp_mode ==
                             PlotTimestampMode::kCustomField);

  auto* remove_button = MakeDestructiveFlatActionButton(tr("Remove series"), box);
  form->addRow(remove_button);

  connect(channel_combo, &QComboBox::currentTextChanged, box,
          [this, index](const QString& text) {
            if (index < config_.series.size()) {
              config_.series[index].channel = text;
              emit configChanged();
            }
          });
  connect(field_edit, &QLineEdit::textChanged, box, [this, index](const QString& text) {
    if (index < config_.series.size()) {
      config_.series[index].field_path = text;
      emit configChanged();
    }
  });
  connect(x_field_edit, &QLineEdit::textChanged, box, [this, index](const QString& text) {
    if (index < config_.series.size()) {
      config_.series[index].x_field_path = text;
      emit configChanged();
    }
  });
  connect(label_edit, &QLineEdit::textChanged, box, [this, index](const QString& text) {
    if (index < config_.series.size()) {
      config_.series[index].label = text;
      emit configChanged();
    }
  });
  connect(color_button, &QPushButton::clicked, box, [this, index, color_button]() {
    if (index >= config_.series.size()) {
      return;
    }
    const QColor chosen = QColorDialog::getColor(config_.series[index].color, this);
    if (!chosen.isValid()) {
      return;
    }
    config_.series[index].color = chosen;
    UpdateColorButton(color_button, chosen);
    emit configChanged();
  });
  connect(line_size_edit, &QLineEdit::textChanged, box, [this, index](const QString& text) {
    if (index < config_.series.size()) {
      config_.series[index].line_size = text;
      emit configChanged();
    }
  });
  connect(timestamp_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), box,
          [this, index, timestamp_combo, custom_ts_edit](int /*idx*/) {
            if (index < config_.series.size()) {
              config_.series[index].timestamp_mode =
                  static_cast<PlotTimestampMode>(timestamp_combo->currentData().toInt());
              custom_ts_edit->setEnabled(config_.series[index].timestamp_mode ==
                                           PlotTimestampMode::kCustomField);
              emit configChanged();
            }
          });
  connect(custom_ts_edit, &QLineEdit::textChanged, box, [this, index](const QString& text) {
    if (index < config_.series.size()) {
      config_.series[index].custom_timestamp_path = text;
      emit configChanged();
    }
  });
  connect(remove_button, &QPushButton::clicked, box, [this, index]() {
    emit removeSeriesRequested(index);
  });
  return box;
}

void PlotSettingsWidget::rebuildSeriesSection() {
  while (QLayoutItem* item = series_list_layout_->takeAt(0)) {
    if (item->widget() != nullptr) {
      item->widget()->deleteLater();
    }
    delete item;
  }
  for (int i = 0; i < config_.series.size(); ++i) {
    series_list_layout_->addWidget(buildSeriesEditor(i, config_.series[i]));
  }
}

void PlotSettingsWidget::emitConfigChanged() {
  config_.title = title_edit_->text();
  config_.x_axis_mode = static_cast<PlotXAxisMode>(
      x_axis_mode_combo_->currentData().toInt());
  if (message_path_mode_combo_ != nullptr) {
    config_.message_path_mode = static_cast<PlotMessagePathMode>(
        message_path_mode_combo_->currentData().toInt());
  }
  config_.sync_with_other_plots = sync_plots_combo_->currentData().toBool();
  if (x_window_spin_ != nullptr) {
    config_.x_window_sec = x_window_spin_->value();
  }
  if (lock_axis_scales_check_ != nullptr) {
    config_.lock_axis_scales = lock_axis_scales_check_->isChecked();
  }
  if (show_legend_values_check_ != nullptr) {
    config_.show_legend_values = show_legend_values_check_->isChecked();
  }
  emit configChanged();
}

void PlotSettingsWidget::onAddSeriesClicked() { emit addSeriesRequested(); }

void PlotSettingsWidget::onRemoveSeriesClicked() {}

}  // namespace plot
}  // namespace autoviz

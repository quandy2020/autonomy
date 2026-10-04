/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_settings_widget.hpp"

#include <algorithm>
#include <functional>

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFontMetrics>
#include <QFormLayout>
#include <QHash>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QPoint>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_manager.hpp"
#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/plot/message_field_tree.hpp"
#include "autoviz/ui/plot/plot_path_utils.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace plot {
namespace {

constexpr char kPlotValueRefreshing[] = "plot_value_refreshing";

QLineEdit* ValuePathLineEdit(QComboBox* value_combo) {
  return value_combo != nullptr ? value_combo->lineEdit() : nullptr;
}

bool ShouldDrillPlotValuePath(common::VisualizationManager* manager, const QString& channel,
                              const QString& field_path) {
  if (channel.trimmed().isEmpty() || field_path.trimmed().isEmpty()) {
    return false;
  }
  const std::string message_type = MessageTypeForChannel(manager, channel);
  return !PlotNextLevelFieldPaths(message_type, field_path).isEmpty();
}

bool IsCompletePlotValuePath(common::VisualizationManager* manager, const QString& channel,
                             const QString& field_path) {
  return IsPlottablePlotValuePath(channel, field_path) &&
         !ShouldDrillPlotValuePath(manager, channel, field_path);
}

void UpdateValueValidation(common::VisualizationManager* manager, QComboBox* value_combo,
                           const QString& channel, const QString& field_path) {
  QLineEdit* line_edit = ValuePathLineEdit(value_combo);
  if (line_edit == nullptr) {
    return;
  }
  const bool valid = IsCompletePlotValuePath(manager, channel, field_path);
  line_edit->setStyleSheet(valid ? QString() : style::mark(style::Mark::Invalid));
  line_edit->setToolTip(valid ? QString()
                              : QObject::tr("Select a numeric message field path"));
}

std::function<void()> InstallPlotValuePathEditor(
    QComboBox* value_combo, common::VisualizationManager* manager,
    const std::function<void()>& on_pick) {
  if (value_combo == nullptr) {
    return {};
  }

  value_combo->setEditable(true);
  value_combo->setInsertPolicy(QComboBox::NoInsert);
  value_combo->setMaxVisibleItems(15);

  QLineEdit* line_edit = ValuePathLineEdit(value_combo);
  if (line_edit == nullptr) {
    return {};
  }

  auto* debounce = new QTimer(value_combo);
  debounce->setSingleShot(true);
  debounce->setInterval(60);

  const auto refresh_items = [value_combo, line_edit, manager]() {
    if (!line_edit->hasFocus()) {
      return;
    }
    const QString text = line_edit->text();
    const QStringList suggestions = PlotValuePathSuggestions(manager, text);

    value_combo->setProperty(kPlotValueRefreshing, true);
    {
      QSignalBlocker blocker(value_combo);
      const int cursor = line_edit->cursorPosition();
      value_combo->clear();
      if (!suggestions.isEmpty()) {
        value_combo->addItems(suggestions);
      }
      line_edit->setText(text);
      line_edit->setCursorPosition(cursor);
    }
    value_combo->setProperty(kPlotValueRefreshing, false);

    if (suggestions.isEmpty()) {
      value_combo->hidePopup();
      return;
    }
    QTimer::singleShot(0, value_combo, [value_combo]() { value_combo->showPopup(); });
  };

  const auto apply_choice = [manager, value_combo, line_edit, on_pick, refresh_items](
                                const QString& choice) {
    if (value_combo->property(kPlotValueRefreshing).toBool()) {
      return;
    }

    QSignalBlocker blocker(value_combo);
    QString next = choice.trimmed();
    QString channel;
    QString field_path;
    SplitPlotValuePath(next, AllKnownChannels(manager), &channel, &field_path);

    if (field_path.isEmpty() && !next.endsWith(QLatin1Char('.'))) {
      next += QLatin1Char('.');
    }

    line_edit->setText(next);
    line_edit->setCursorPosition(next.size());

    if (on_pick) {
      on_pick();
    }

    SplitPlotValuePath(next, AllKnownChannels(manager), &channel, &field_path);
    if (ShouldDrillPlotValuePath(manager, channel, field_path)) {
      if (!next.endsWith(QLatin1Char('.'))) {
        next += QLatin1Char('.');
        line_edit->setText(next);
        line_edit->setCursorPosition(next.size());
      }
      refresh_items();
      return;
    }

    value_combo->hidePopup();
  };

  QObject::connect(debounce, &QTimer::timeout, value_combo, refresh_items);
  QObject::connect(line_edit, &QLineEdit::textChanged, value_combo,
                   [debounce, refresh_items, value_combo](const QString& text) {
                     if (text.endsWith(QLatin1Char('.'))) {
                       debounce->stop();
                       QTimer::singleShot(0, value_combo, refresh_items);
                     } else {
                       debounce->start();
                     }
                   });
  QObject::connect(value_combo, QOverload<int>::of(&QComboBox::activated), value_combo,
                   [value_combo, apply_choice](int index) {
                     if (value_combo->property(kPlotValueRefreshing).toBool()) {
                       return;
                     }
                     apply_choice(value_combo->itemText(index));
                   });

  return refresh_items;
}

QString ColorSwatchStyle(const QColor& color) {
  const QString css = color.isValid() ? color.name(QColor::HexRgb)
                                      : QStringLiteral("#4e98e2");
  return style::mark(style::Mark::Swatch, css);
}

QPushButton* MakeColorSwatch(const QColor& color, QWidget* parent) {
  auto* button = new QPushButton(parent);
  button->setFixedSize(14, 14);
  button->setFlat(true);
  button->setCursor(Qt::PointingHandCursor);
  button->setStyleSheet(ColorSwatchStyle(color));
  return button;
}

QWidget* MakeSegmentedToggle(QWidget* parent, const QStringList& labels,
                             int checked_index,
                             const std::function<void(int)>& on_changed) {
  auto* container = new QWidget(parent);
  container->setObjectName(QStringLiteral("plot_segmented_toggle"));
  container->setAttribute(Qt::WA_StyledBackground, true);
  auto* layout = new QHBoxLayout(container);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  container->setFixedHeight(24);
  container->setStyleSheet(
      style::sheet(QStringLiteral("plot/segmented_toggle_inline"),
                   style::tokens()));

  int min_button_w = 48;
  {
    QFontMetrics fm(container->font());
    for (const QString& label : labels) {
      min_button_w = std::max(min_button_w, fm.horizontalAdvance(label) + 20);
    }
  }

  for (int i = 0; i < labels.size(); ++i) {
    auto* button = new QPushButton(labels.at(i), container);
    button->setCheckable(true);
    button->setChecked(i == checked_index);
    button->setAutoExclusive(true);
    button->setCursor(Qt::PointingHandCursor);
    button->setMinimumWidth(min_button_w);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    layout->addWidget(button, 1);
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
  setMinimumWidth(260);
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
  y_auto_scale_check_ = new QCheckBox(tr("Auto-scale Y"), y_axis_body);
  y_auto_scale_check_->setChecked(config_.y_auto_scale);
  y_axis_layout->addWidget(y_auto_scale_check_);
  connect(y_auto_scale_check_, &QCheckBox::toggled, this, [this](bool checked) {
    if (y_min_spin_ != nullptr) {
      y_min_spin_->setEnabled(!checked);
    }
    if (y_max_spin_ != nullptr) {
      y_max_spin_->setEnabled(!checked);
    }
    emitConfigChanged();
  });

  auto* y_range_form = new QFormLayout();
  y_range_form->setContentsMargins(0, 0, 0, 0);
  y_range_form->setSpacing(6);
  y_min_spin_ = new QDoubleSpinBox(y_axis_body);
  y_min_spin_->setRange(-1.0e9, 1.0e9);
  y_min_spin_->setDecimals(4);
  y_min_spin_->setValue(config_.y_min);
  y_min_spin_->setEnabled(!config_.y_auto_scale);
  StyleCompactSettingsField(y_min_spin_);
  y_range_form->addRow(tr("Y min"), y_min_spin_);
  y_max_spin_ = new QDoubleSpinBox(y_axis_body);
  y_max_spin_->setRange(-1.0e9, 1.0e9);
  y_max_spin_->setDecimals(4);
  y_max_spin_->setValue(config_.y_max);
  y_max_spin_->setEnabled(!config_.y_auto_scale);
  StyleCompactSettingsField(y_max_spin_);
  y_range_form->addRow(tr("Y max"), y_max_spin_);
  y_axis_layout->addLayout(y_range_form);
  connect(y_min_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &PlotSettingsWidget::emitConfigChanged);
  connect(y_max_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &PlotSettingsWidget::emitConfigChanged);

  show_grid_check_ = new QCheckBox(tr("Show grid"), y_axis_body);
  show_grid_check_->setChecked(config_.show_grid);
  y_axis_layout->addWidget(show_grid_check_);
  connect(show_grid_check_, &QCheckBox::toggled, this,
          &PlotSettingsWidget::emitConfigChanged);

  show_reference_y_check_ = new QCheckBox(tr("Reference line"), y_axis_body);
  show_reference_y_check_->setChecked(config_.show_reference_y);
  y_axis_layout->addWidget(show_reference_y_check_);
  connect(show_reference_y_check_, &QCheckBox::toggled, this, [this](bool checked) {
    if (reference_y_spin_ != nullptr) {
      reference_y_spin_->setEnabled(checked);
    }
    emitConfigChanged();
  });
  reference_y_spin_ = new QDoubleSpinBox(y_axis_body);
  reference_y_spin_->setRange(-1.0e9, 1.0e9);
  reference_y_spin_->setDecimals(4);
  reference_y_spin_->setValue(config_.reference_y);
  reference_y_spin_->setEnabled(config_.show_reference_y);
  StyleCompactSettingsField(reference_y_spin_);
  auto* ref_form = new QFormLayout();
  ref_form->setContentsMargins(0, 0, 0, 0);
  ref_form->addRow(tr("Y value"), reference_y_spin_);
  y_axis_layout->addLayout(ref_form);
  connect(reference_y_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &PlotSettingsWidget::emitConfigChanged);

  lock_axis_scales_check_ = new QCheckBox(tr("Locked axis scales (1:1)"), y_axis_body);
  lock_axis_scales_check_->setChecked(config_.lock_axis_scales);
  y_axis_layout->addWidget(lock_axis_scales_check_);
  connect(lock_axis_scales_check_, &QCheckBox::toggled, this,
          &PlotSettingsWidget::emitConfigChanged);

  y_auto_scale_right_check_ =
      new QCheckBox(tr("Auto-scale right Y"), y_axis_body);
  y_auto_scale_right_check_->setChecked(config_.y_auto_scale_right);
  y_axis_layout->addWidget(y_auto_scale_right_check_);
  connect(y_auto_scale_right_check_, &QCheckBox::toggled, this,
          [this](bool checked) {
            if (y_min_right_spin_ != nullptr) {
              y_min_right_spin_->setEnabled(!checked);
            }
            if (y_max_right_spin_ != nullptr) {
              y_max_right_spin_->setEnabled(!checked);
            }
            emitConfigChanged();
          });
  auto* y_right_form = new QFormLayout();
  y_right_form->setContentsMargins(0, 0, 0, 0);
  y_right_form->setSpacing(6);
  y_min_right_spin_ = new QDoubleSpinBox(y_axis_body);
  y_min_right_spin_->setRange(-1.0e9, 1.0e9);
  y_min_right_spin_->setDecimals(4);
  y_min_right_spin_->setValue(config_.y_min_right);
  y_min_right_spin_->setEnabled(!config_.y_auto_scale_right);
  StyleCompactSettingsField(y_min_right_spin_);
  y_right_form->addRow(tr("Right Y min"), y_min_right_spin_);
  y_max_right_spin_ = new QDoubleSpinBox(y_axis_body);
  y_max_right_spin_->setRange(-1.0e9, 1.0e9);
  y_max_right_spin_->setDecimals(4);
  y_max_right_spin_->setValue(config_.y_max_right);
  y_max_right_spin_->setEnabled(!config_.y_auto_scale_right);
  StyleCompactSettingsField(y_max_right_spin_);
  y_right_form->addRow(tr("Right Y max"), y_max_right_spin_);
  y_axis_layout->addLayout(y_right_form);
  connect(y_min_right_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &PlotSettingsWidget::emitConfigChanged);
  connect(y_max_right_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &PlotSettingsWidget::emitConfigChanged);

  outer->addWidget(makeCollapsibleSection(tr("Y-axis"), y_axis_body, false));

  updateAxisModeVisibility();

  series_container_ = new QWidget(this);
  series_list_layout_ = new QVBoxLayout(series_container_);
  series_list_layout_->setContentsMargins(0, 0, 0, 0);
  series_list_layout_->setSpacing(6);

  auto* series_toolbar = new QWidget(this);
  auto* series_toolbar_layout = new QHBoxLayout(series_toolbar);
  series_toolbar_layout->setContentsMargins(0, 0, 0, 0);
  series_toolbar_layout->setSpacing(6);
  series_filter_combo_ = new QComboBox(series_toolbar);
  series_filter_combo_->addItem(tr("Show all"));
  series_filter_combo_->addItem(tr("Visible"));
  series_filter_combo_->addItem(tr("Hidden"));
  StyleCompactSettingsField(series_filter_combo_);
  connect(series_filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int /*index*/) { applySeriesFilter(); });
  add_series_button_ = MakeFlatActionButton(QStringLiteral("+"), series_toolbar);
  add_series_button_->setToolTip(tr("Add series"));
  add_series_button_->setFixedSize(22, 22);
  add_series_button_->setStyleSheet(
      style::sheet(QStringLiteral("plot/add_series_button")));
  connect(add_series_button_, &QPushButton::clicked, this,
          &PlotSettingsWidget::onAddSeriesClicked);
  series_toolbar_layout->addWidget(series_filter_combo_, 1);
  series_toolbar_layout->addWidget(add_series_button_, 0, Qt::AlignVCenter);

  auto* series_body = new QWidget(this);
  auto* series_body_layout = new QVBoxLayout(series_body);
  series_body_layout->setContentsMargins(0, 0, 0, 0);
  series_body_layout->setSpacing(4);
  series_body_layout->addWidget(series_toolbar);
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
  if (y_auto_scale_check_ != nullptr) {
    y_auto_scale_check_->setChecked(config_.y_auto_scale);
  }
  if (y_min_spin_ != nullptr) {
    y_min_spin_->setValue(config_.y_min);
    y_min_spin_->setEnabled(!config_.y_auto_scale);
  }
  if (y_max_spin_ != nullptr) {
    y_max_spin_->setValue(config_.y_max);
    y_max_spin_->setEnabled(!config_.y_auto_scale);
  }
  if (y_auto_scale_right_check_ != nullptr) {
    y_auto_scale_right_check_->setChecked(config_.y_auto_scale_right);
  }
  if (y_min_right_spin_ != nullptr) {
    y_min_right_spin_->setValue(config_.y_min_right);
    y_min_right_spin_->setEnabled(!config_.y_auto_scale_right);
  }
  if (y_max_right_spin_ != nullptr) {
    y_max_right_spin_->setValue(config_.y_max_right);
    y_max_right_spin_->setEnabled(!config_.y_auto_scale_right);
  }
  if (show_grid_check_ != nullptr) {
    show_grid_check_->setChecked(config_.show_grid);
  }
  if (show_reference_y_check_ != nullptr) {
    show_reference_y_check_->setChecked(config_.show_reference_y);
  }
  if (reference_y_spin_ != nullptr) {
    reference_y_spin_->setValue(config_.reference_y);
    reference_y_spin_->setEnabled(config_.show_reference_y);
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

QString SeriesCardTitle(const QString& channel) {
  const QString trimmed = channel.trimmed();
  return trimmed.isEmpty() ? QObject::tr("(no channel)") : trimmed;
}

void PlotSettingsWidget::refreshChannelLists() {
  for (int i = 0; i < series_list_layout_->count(); ++i) {
    QLayoutItem* item = series_list_layout_->itemAt(i);
    if (item == nullptr || item->widget() == nullptr) {
      continue;
    }
    auto* value_combo =
        item->widget()->findChild<QComboBox*>(QStringLiteral("plot_series_value_combo"));
    if (value_combo == nullptr) {
      continue;
    }
    QLineEdit* line_edit = ValuePathLineEdit(value_combo);
    if (line_edit == nullptr || line_edit->hasFocus()) {
      continue;
    }
    refreshSeriesValueEdit(value_combo);
  }
}

QStringList PlotSettingsWidget::knownChannels() const {
  return AllKnownChannels(manager_);
}

void PlotSettingsWidget::refreshSeriesValueEdit(QComboBox* value_combo) {
  if (value_combo == nullptr) {
    return;
  }
  QLineEdit* line_edit = ValuePathLineEdit(value_combo);
  if (line_edit == nullptr) {
    return;
  }
  const QString current = line_edit->text().trimmed();
  QString channel;
  QString field_path;
  SplitPlotValuePath(current, knownChannels(), &channel, &field_path);
  UpdateValueValidation(manager_, value_combo, channel, field_path);
}

void PlotSettingsWidget::applySeriesFilter() {
  if (series_filter_combo_ == nullptr || series_list_layout_ == nullptr) {
    return;
  }
  const int mode = series_filter_combo_->currentIndex();
  for (int i = 0; i < series_list_layout_->count(); ++i) {
    QLayoutItem* item = series_list_layout_->itemAt(i);
    if (item == nullptr || item->widget() == nullptr || i >= config_.series.size()) {
      continue;
    }
    bool visible = true;
    if (mode == 1) {
      visible = config_.series[i].enabled;
    } else if (mode == 2) {
      visible = !config_.series[i].enabled;
    }
    item->widget()->setVisible(visible);
  }
}

QWidget* PlotSettingsWidget::buildSeriesEditor(int index,
                                               const PlotSeriesConfig& series) {
  auto* card = new QFrame(series_container_);
  card->setObjectName(QStringLiteral("plot_series_card"));
  card->setAttribute(Qt::WA_StyledBackground, true);
  card->setStyleSheet(
      style::sheet(QStringLiteral("plot/series_card"), style::tokens()));
  auto* card_layout = new QVBoxLayout(card);
  card_layout->setContentsMargins(8, 6, 8, 6);
  card_layout->setSpacing(4);

  auto* header = new QHBoxLayout();
  header->setContentsMargins(0, 0, 0, 0);
  header->setSpacing(6);
  auto* color_swatch = MakeColorSwatch(series.color, card);
  auto* title_label = new QLabel(SeriesCardTitle(series.channel), card);
  title_label->setObjectName(QStringLiteral("plot_series_title"));
  title_label->setStyleSheet(
      style::sheet(QStringLiteral("plot/series_title"), style::tokens()));
  title_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  title_label->setToolTip(series.channel.isEmpty() ? tr("Select a channel in Value")
                                                   : series.channel);
  auto* visibility_button = new QToolButton(card);
  visibility_button->setObjectName(QStringLiteral("plot_series_visibility"));
  visibility_button->setCheckable(true);
  visibility_button->setChecked(series.enabled);
  visibility_button->setText(series.enabled ? QStringLiteral("◉") : QStringLiteral("○"));
  visibility_button->setToolTip(tr("Toggle series visibility"));
  visibility_button->setAutoRaise(true);
  visibility_button->setFixedSize(20, 20);
  auto* remove_button = new QToolButton(card);
  remove_button->setObjectName(QStringLiteral("plot_series_remove"));
  remove_button->setText(QStringLiteral("×"));
  remove_button->setToolTip(tr("Remove series"));
  remove_button->setAutoRaise(true);
  remove_button->setFixedSize(20, 20);
  header->addWidget(color_swatch, 0, Qt::AlignVCenter);
  header->addWidget(title_label, 1);
  header->addWidget(visibility_button, 0, Qt::AlignVCenter);
  header->addWidget(remove_button, 0, Qt::AlignVCenter);
  card_layout->addLayout(header);

  auto* form = new QFormLayout();
  ApplyCompactForm(form);

  auto* value_box = new QWidget(card);
  auto* value_layout = new QHBoxLayout(value_box);
  value_layout->setContentsMargins(0, 0, 0, 0);
  value_layout->setSpacing(4);

  auto* value_combo = new QComboBox(value_box);
  value_combo->setObjectName(QStringLiteral("plot_series_value_combo"));
  value_combo->setEditable(true);
  value_combo->setInsertPolicy(QComboBox::NoInsert);
  value_combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  value_combo->setMinimumWidth(120);
  StyleCompactSettingsField(value_combo);
  const QString combined = CombinedPlotValuePath(series.channel, series.field_path);
  if (!combined.isEmpty()) {
    value_combo->setEditText(combined);
  } else if (QLineEdit* line_edit = value_combo->lineEdit()) {
    line_edit->setPlaceholderText(tr("/channel.field"));
  }

  auto* channel_button = new QToolButton(value_box);
  channel_button->setText(QStringLiteral("▾"));
  channel_button->setToolTip(tr("Browse channels"));
  channel_button->setAutoRaise(true);
  channel_button->setFixedSize(20, 20);
  value_layout->addWidget(value_combo, 1);
  value_layout->addWidget(channel_button, 0, Qt::AlignVCenter);
  UpdateValueValidation(manager_, value_combo, series.channel, series.field_path);
  form->addRow(MakeSettingsFormLabel(tr("Value"), card), value_box);

  auto* label_edit = new QLineEdit(series.label, card);
  StyleCompactSettingsField(label_edit);
  form->addRow(MakeSettingsFormLabel(tr("Label"), card), label_edit);

  auto* color_button = new QPushButton(card);
  UpdateColorButton(color_button, series.color);
  form->addRow(MakeSettingsFormLabel(tr("Color"), card), color_button);

  auto* line_size_edit = new QLineEdit(series.line_size, card);
  StyleCompactSettingsField(line_size_edit);
  form->addRow(MakeSettingsFormLabel(tr("Line size"), card), line_size_edit);

  auto* show_line = MakeSegmentedToggle(
      card, {tr("Off"), tr("On")}, series.show_line ? 1 : 0,
      [this, index](int selected) {
        if (index >= 0 && index < config_.series.size()) {
          config_.series[index].show_line = selected == 1;
          emit configChanged();
        }
      });
  form->addRow(MakeSettingsFormLabel(tr("Show line"), card), show_line);

  auto* y_axis_toggle = MakeSegmentedToggle(
      card, {tr("Left"), tr("Right")}, series.use_right_y ? 1 : 0,
      [this, index](int selected) {
        if (index < config_.series.size()) {
          config_.series[index].use_right_y = selected == 1;
          emit configChanged();
        }
      });
  form->addRow(MakeSettingsFormLabel(tr("Y axis"), card), y_axis_toggle);

  auto* binary_op_combo = new QComboBox(card);
  binary_op_combo->addItem(tr("None"), static_cast<int>(PlotBinaryOp::kNone));
  binary_op_combo->addItem(tr("a + b"), static_cast<int>(PlotBinaryOp::kAdd));
  binary_op_combo->addItem(tr("a − b"), static_cast<int>(PlotBinaryOp::kSub));
  binary_op_combo->addItem(tr("a × b"), static_cast<int>(PlotBinaryOp::kMul));
  binary_op_combo->addItem(tr("a ÷ b"), static_cast<int>(PlotBinaryOp::kDiv));
  const int binary_index =
      binary_op_combo->findData(static_cast<int>(series.binary_op));
  binary_op_combo->setCurrentIndex(binary_index >= 0 ? binary_index : 0);
  StyleCompactSettingsField(binary_op_combo);
  form->addRow(MakeSettingsFormLabel(tr("Binary op"), card), binary_op_combo);

  auto* secondary_combo = new QComboBox(card);
  secondary_combo->setObjectName(QStringLiteral("plot_series_secondary_combo"));
  secondary_combo->setEditable(true);
  secondary_combo->setInsertPolicy(QComboBox::NoInsert);
  secondary_combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  secondary_combo->setMinimumWidth(120);
  StyleCompactSettingsField(secondary_combo);
  const QString secondary_combined = CombinedPlotValuePath(
      series.secondary_channel.isEmpty() ? series.channel : series.secondary_channel,
      series.secondary_field_path);
  if (!series.secondary_field_path.isEmpty() || !series.secondary_channel.isEmpty()) {
    secondary_combo->setEditText(secondary_combined);
  } else if (QLineEdit* line_edit = secondary_combo->lineEdit()) {
    line_edit->setPlaceholderText(tr("/channel.field (operand b)"));
  }
  form->addRow(MakeSettingsFormLabel(tr("Operand b"), card), secondary_combo);

  auto* max_dt_spin = new QDoubleSpinBox(card);
  max_dt_spin->setRange(0.0, 10.0);
  max_dt_spin->setDecimals(3);
  max_dt_spin->setSingleStep(0.01);
  max_dt_spin->setSuffix(tr(" s"));
  max_dt_spin->setValue(series.binary_max_dt_sec);
  StyleCompactSettingsField(max_dt_spin);
  form->addRow(MakeSettingsFormLabel(tr("Match Δt"), card), max_dt_spin);

  const bool binary_enabled = series.binary_op != PlotBinaryOp::kNone;
  secondary_combo->setEnabled(binary_enabled);
  max_dt_spin->setEnabled(binary_enabled);

  auto* timestamp_combo = new QComboBox(card);
  timestamp_combo->addItem(tr("Log time"),
                           static_cast<int>(PlotTimestampMode::kLogTime));
  timestamp_combo->addItem(tr("Receive time"),
                           static_cast<int>(PlotTimestampMode::kReceiveTime));
  timestamp_combo->addItem(tr("Custom field"),
                           static_cast<int>(PlotTimestampMode::kCustomField));
  const int ts_index = timestamp_combo->findData(static_cast<int>(series.timestamp_mode));
  timestamp_combo->setCurrentIndex(ts_index >= 0 ? ts_index : 0);
  StyleCompactSettingsField(timestamp_combo);
  form->addRow(MakeSettingsFormLabel(tr("Timestamp"), card), timestamp_combo);

  auto* custom_ts_edit = new QLineEdit(series.custom_timestamp_path, card);
  custom_ts_edit->setPlaceholderText(tr("e.g. header.stamp"));
  StyleCompactSettingsField(custom_ts_edit);
  form->addRow(MakeSettingsFormLabel(tr("Custom timestamp"), card), custom_ts_edit);
  custom_ts_edit->setEnabled(series.timestamp_mode == PlotTimestampMode::kCustomField);

  auto* x_field_edit = new QLineEdit(series.x_field_path, card);
  x_field_edit->setPlaceholderText(tr("X-value path (message path mode)"));
  StyleCompactSettingsField(x_field_edit);
  form->addRow(MakeSettingsFormLabel(tr("X-value path"), card), x_field_edit);

  card_layout->addLayout(form);

  QLineEdit* value_line_edit = ValuePathLineEdit(value_combo);

  const auto sync_value_path_display = [this, title_label, value_combo](
                                             const QString& channel,
                                             const QString& field_path) {
    UpdateValueValidation(manager_, value_combo, channel, field_path);
    title_label->setText(SeriesCardTitle(channel));
    title_label->setToolTip(channel.isEmpty() ? tr("Select a channel in Value")
                                              : channel);
  };

  const auto commit_value_path = [this, index, title_label, value_combo, value_line_edit,
                                  sync_value_path_display]() {
    if (index >= config_.series.size() || value_line_edit == nullptr) {
      return;
    }
    QString channel;
    QString field_path;
    const QString combined_path = value_line_edit->text().trimmed();
    SplitPlotValuePath(combined_path, knownChannels(), &channel, &field_path);
    sync_value_path_display(channel, field_path);
    if (config_.series[index].channel == channel &&
        config_.series[index].field_path == field_path) {
      return;
    }
    config_.series[index].channel = channel;
    config_.series[index].field_path = field_path;
    emit configChanged();
  };

  if (value_line_edit != nullptr) {
    connect(value_line_edit, &QLineEdit::textChanged, card,
            [this, sync_value_path_display](const QString& text) {
              QString channel;
              QString field_path;
              SplitPlotValuePath(text.trimmed(), knownChannels(), &channel, &field_path);
              sync_value_path_display(channel, field_path);
            });
    connect(value_line_edit, &QLineEdit::editingFinished, card, commit_value_path);
  }
  const std::function<void()> refresh_value_suggestions =
      InstallPlotValuePathEditor(value_combo, manager_, commit_value_path);
  connect(channel_button, &QToolButton::clicked, card,
          [value_combo, value_line_edit, commit_value_path, refresh_value_suggestions,
           channel_button, this]() {
            QMenu menu(value_combo);
            const QStringList channels = AllKnownChannels(manager_);
            for (const QString& channel : channels) {
              menu.addAction(channel, [value_combo, value_line_edit, commit_value_path,
                                       refresh_value_suggestions, channel]() {
                const QString next = channel + QLatin1Char('.');
                if (value_line_edit != nullptr) {
                  value_line_edit->setText(next);
                  value_line_edit->setCursorPosition(next.size());
                  value_line_edit->setFocus();
                } else {
                  value_combo->setEditText(next);
                }
                commit_value_path();
                refresh_value_suggestions();
              });
            }
            menu.exec(channel_button->mapToGlobal(QPoint(0, channel_button->height())));
          });
  connect(visibility_button, &QToolButton::toggled, card, [this, index, visibility_button](bool checked) {
    if (index < config_.series.size()) {
      config_.series[index].enabled = checked;
      visibility_button->setText(checked ? QStringLiteral("◉") : QStringLiteral("○"));
      emit configChanged();
      applySeriesFilter();
    }
  });
  connect(remove_button, &QToolButton::clicked, card,
          [this, index]() { emit removeSeriesRequested(index); });
  connect(color_swatch, &QPushButton::clicked, card, [this, index, color_swatch, color_button]() {
    if (index >= config_.series.size()) {
      return;
    }
    const QColor chosen = QColorDialog::getColor(config_.series[index].color, this);
    if (!chosen.isValid()) {
      return;
    }
    config_.series[index].color = chosen;
    color_swatch->setStyleSheet(ColorSwatchStyle(chosen));
    UpdateColorButton(color_button, chosen);
    emit configChanged();
  });
  connect(label_edit, &QLineEdit::textChanged, card, [this, index](const QString& text) {
    if (index < config_.series.size()) {
      config_.series[index].label = text;
      emit configChanged();
    }
  });
  connect(color_button, &QPushButton::clicked, card, [this, index, color_swatch, color_button]() {
    if (index >= config_.series.size()) {
      return;
    }
    const QColor chosen = QColorDialog::getColor(config_.series[index].color, this);
    if (!chosen.isValid()) {
      return;
    }
    config_.series[index].color = chosen;
    color_swatch->setStyleSheet(ColorSwatchStyle(chosen));
    UpdateColorButton(color_button, chosen);
    emit configChanged();
  });
  connect(line_size_edit, &QLineEdit::textChanged, card, [this, index](const QString& text) {
    if (index < config_.series.size()) {
      config_.series[index].line_size = text;
      emit configChanged();
    }
  });
  connect(binary_op_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), card,
          [this, index, binary_op_combo, secondary_combo, max_dt_spin](int /*idx*/) {
            if (index >= config_.series.size()) {
              return;
            }
            const auto op = static_cast<PlotBinaryOp>(
                binary_op_combo->currentData().toInt());
            config_.series[index].binary_op = op;
            const bool enabled = op != PlotBinaryOp::kNone;
            secondary_combo->setEnabled(enabled);
            max_dt_spin->setEnabled(enabled);
            emit configChanged();
          });
  connect(max_dt_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), card,
          [this, index](double value) {
            if (index < config_.series.size()) {
              config_.series[index].binary_max_dt_sec = value;
              emit configChanged();
            }
          });

  QLineEdit* secondary_line_edit = ValuePathLineEdit(secondary_combo);
  const auto commit_secondary_path =
      [this, index, secondary_line_edit]() {
        if (index >= config_.series.size() || secondary_line_edit == nullptr) {
          return;
        }
        QString channel;
        QString field_path;
        SplitPlotValuePath(secondary_line_edit->text().trimmed(), knownChannels(),
                           &channel, &field_path);
        if (config_.series[index].secondary_channel == channel &&
            config_.series[index].secondary_field_path == field_path) {
          return;
        }
        config_.series[index].secondary_channel = channel;
        config_.series[index].secondary_field_path = field_path;
        emit configChanged();
      };
  if (secondary_line_edit != nullptr) {
    connect(secondary_line_edit, &QLineEdit::editingFinished, card,
            commit_secondary_path);
  }
  InstallPlotValuePathEditor(secondary_combo, manager_, commit_secondary_path);

  connect(timestamp_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), card,
          [this, index, timestamp_combo, custom_ts_edit](int /*idx*/) {
            if (index < config_.series.size()) {
              config_.series[index].timestamp_mode =
                  static_cast<PlotTimestampMode>(timestamp_combo->currentData().toInt());
              custom_ts_edit->setEnabled(config_.series[index].timestamp_mode ==
                                           PlotTimestampMode::kCustomField);
              emit configChanged();
            }
          });
  connect(custom_ts_edit, &QLineEdit::textChanged, card, [this, index](const QString& text) {
    if (index < config_.series.size()) {
      config_.series[index].custom_timestamp_path = text;
      emit configChanged();
    }
  });
  connect(x_field_edit, &QLineEdit::textChanged, card, [this, index](const QString& text) {
    if (index < config_.series.size()) {
      config_.series[index].x_field_path = text;
      emit configChanged();
    }
  });
  return card;
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
  applySeriesFilter();
  PolishSettingsFields(series_container_);
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
  if (y_auto_scale_check_ != nullptr) {
    config_.y_auto_scale = y_auto_scale_check_->isChecked();
  }
  if (y_min_spin_ != nullptr) {
    config_.y_min = y_min_spin_->value();
  }
  if (y_max_spin_ != nullptr) {
    config_.y_max = y_max_spin_->value();
  }
  if (config_.y_max <= config_.y_min) {
    config_.y_max = config_.y_min + 1.0;
  }
  if (y_auto_scale_right_check_ != nullptr) {
    config_.y_auto_scale_right = y_auto_scale_right_check_->isChecked();
  }
  if (y_min_right_spin_ != nullptr) {
    config_.y_min_right = y_min_right_spin_->value();
  }
  if (y_max_right_spin_ != nullptr) {
    config_.y_max_right = y_max_right_spin_->value();
  }
  if (config_.y_max_right <= config_.y_min_right) {
    config_.y_max_right = config_.y_min_right + 1.0;
  }
  if (show_grid_check_ != nullptr) {
    config_.show_grid = show_grid_check_->isChecked();
  }
  if (show_reference_y_check_ != nullptr) {
    config_.show_reference_y = show_reference_y_check_->isChecked();
  }
  if (reference_y_spin_ != nullptr) {
    config_.reference_y = reference_y_spin_->value();
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

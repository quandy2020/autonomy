/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/app_settings_dialog.hpp"

#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

#include "autoviz/common/display_property.hpp"
#include "autoviz/common/transformation_manager.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/transform/buffer.hpp"
#include "autoviz/ui/app_preferences.hpp"
#include "autoviz/ui/app_shortcuts_editor_widget.hpp"
#include "autoviz/ui/app_theme.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace {

QStringList CollectTfFrames(common::VisualizationManager* manager) {
  QStringList frames;
  if (manager == nullptr || manager->tfBuffer() == nullptr) {
    return frames;
  }
  for (const transform::TfFrameStats& stats : manager->tfBuffer()->frameStats()) {
    const QString frame = QString::fromStdString(stats.frame_id).trimmed();
    if (!frame.isEmpty()) {
      frames.push_back(frame);
    }
  }
  frames.sort(Qt::CaseInsensitive);
  frames.removeDuplicates();
  return frames;
}

void AddComboItemIfMissing(QComboBox* combo, const QString& value) {
  if (combo == nullptr || value.isEmpty()) {
    return;
  }
  if (combo->findText(value, Qt::MatchFixedString) < 0) {
    combo->addItem(value);
  }
}

QSpinBox* MakeRgbSpinBox(int value, QWidget* parent) {
  auto* spin = new QSpinBox(parent);
  spin->setRange(0, 255);
  spin->setValue(value);
  spin->setFixedWidth(56);
  spin->setAlignment(Qt::AlignCenter);
  return spin;
}

}  // namespace

AppSettingsDialog::AppSettingsDialog(common::VisualizationManager* manager,
                                     QWidget* parent)
    : manager_(manager), QDialog(parent) {
  setWindowTitle(tr("Settings — Autoviz"));
  setModal(true);
  resize(580, 720);
  setMinimumSize(520, 560);

  const AppUiPreferences ui_prefs = LoadAppUiPreferences();

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  SettingsScrollForInspector(scroll);

  auto* content = new QWidget(scroll);
  ApplyPanelShell(content);
  auto* body = new QVBoxLayout(content);
  body->setContentsMargins(14, 14, 14, 10);
  body->setSpacing(10);

  auto* title = new QLabel(tr("Application Settings"), content);
  StyleSectionTitle(title);
  body->addWidget(title);

  auto* subtitle = new QLabel(
      tr("Configure language, appearance, shortcuts, visualization, rendering, "
         "playback, and window layout."),
      content);
  subtitle->setWordWrap(true);
  StyleHintLabel(subtitle);
  body->addWidget(subtitle);

  auto* language_body = new QWidget(content);
  auto* language_form = new QFormLayout(language_body);
  ApplyCompactForm(language_form);
  language_combo_ = new QComboBox(language_body);
  language_combo_->setMinimumWidth(240);
  language_combo_->addItem(tr("Follow system"), QString());
  language_combo_->addItem(tr("English"), QStringLiteral("en"));
  language_combo_->addItem(tr("简体中文"), QStringLiteral("zh_CN"));
  language_combo_->addItem(tr("繁體中文"), QStringLiteral("zh_TW"));
  language_combo_->addItem(tr("日本語"), QStringLiteral("ja_JP"));
  language_combo_->addItem(tr("한국어"), QStringLiteral("ko_KR"));
  language_combo_->addItem(tr("Deutsch"), QStringLiteral("de_DE"));
  language_combo_->addItem(tr("Français"), QStringLiteral("fr_FR"));
  language_combo_->addItem(tr("Русский"), QStringLiteral("ru_RU"));
  language_combo_->addItem(tr("Español"), QStringLiteral("es_ES"));
  language_form->addRow(tr("Interface language"), language_combo_);
  auto* language_hint =
      new QLabel(tr("Changing language reloads translations immediately. "
                    "Restart Autoviz if some labels stay in the previous language."),
                 language_body);
  language_hint->setWordWrap(true);
  StyleHintLabel(language_hint);
  language_form->addRow(QString(), language_hint);
  body->addWidget(
      MakeCollapsibleSection(content, tr("Language"), language_body, true));

  auto* appearance_body = new QWidget(content);
  auto* appearance_form = new QFormLayout(appearance_body);
  ApplyCompactForm(appearance_form);
  auto* appearance_hint = new QLabel(
      tr("Autoviz uses the RViz2-style Fusion light interface. "
         "The default 3D viewport background is dark gray (48, 48, 48), "
         "matching RViz2."),
      appearance_body);
  appearance_hint->setWordWrap(true);
  StyleHintLabel(appearance_hint);
  appearance_form->addRow(QString(), appearance_hint);
  auto* apply_theme_bg_button =
      MakeFlatActionButton(tr("Apply suggested 3D background"), appearance_body);
  appearance_form->addRow(QString(), apply_theme_bg_button);
  connect(apply_theme_bg_button, &QPushButton::clicked, this, [this]() {
    syncBackgroundUiFromColor(AppThemeSuggestedViewportBackground());
  });
  body->addWidget(
      MakeCollapsibleSection(content, tr("Appearance"), appearance_body, true));

  auto* shortcuts_body = new QWidget(content);
  auto* shortcuts_layout = new QVBoxLayout(shortcuts_body);
  shortcuts_layout->setContentsMargins(0, 0, 0, 0);
  shortcuts_layout->setSpacing(6);
  shortcuts_editor_ = new ShortcutsEditorWidget(shortcuts_body);
  shortcuts_layout->addWidget(shortcuts_editor_);
  auto* reset_shortcuts_button =
      MakeFlatActionButton(tr("Reset shortcuts to defaults"), shortcuts_body);
  shortcuts_layout->addWidget(reset_shortcuts_button);
  connect(reset_shortcuts_button, &QPushButton::clicked, shortcuts_editor_,
          &ShortcutsEditorWidget::resetToDefaults);
  body->addWidget(MakeCollapsibleSection(content, tr("Keyboard shortcuts"),
                                         shortcuts_body, false));

  auto* general_body = new QWidget(content);
  auto* general_form = new QFormLayout(general_body);
  ApplyCompactForm(general_form);

  fixed_frame_combo_ = new QComboBox(general_body);
  fixed_frame_combo_->setEditable(true);
  fixed_frame_combo_->setInsertPolicy(QComboBox::NoInsert);
  fixed_frame_combo_->setMinimumWidth(220);
  general_form->addRow(tr("Fixed frame"), fixed_frame_combo_);

  transformer_combo_ = new QComboBox(general_body);
  transformer_combo_->setMinimumWidth(220);
  if (manager_ != nullptr) {
    for (const common::PluginInfo& info :
         manager_->transformationManager().availableTransformers()) {
      transformer_combo_->addItem(QString::fromStdString(info.name),
                                   QString::fromStdString(info.class_id));
    }
  }
  general_form->addRow(tr("TF transformer"), transformer_combo_);

  view_controller_combo_ = new QComboBox(general_body);
  view_controller_combo_->addItem(tr("Orbit"), QStringLiteral("Orbit"));
  view_controller_combo_->addItem(tr("XY Orbit"), QStringLiteral("XYOrbit"));
  view_controller_combo_->addItem(tr("Top Down"), QStringLiteral("TopDown"));
  view_controller_combo_->addItem(tr("Top Down Ortho"),
                                  QStringLiteral("TopDownOrtho"));
  view_controller_combo_->addItem(tr("Third Person Follow"),
                                  QStringLiteral("ThirdPersonFollow"));
  view_controller_combo_->addItem(tr("FPS"), QStringLiteral("FPS"));
  general_form->addRow(tr("Default view"), view_controller_combo_);
  body->addWidget(
      MakeCollapsibleSection(content, tr("General"), general_body, true));

  auto* view_body = new QWidget(content);
  auto* view_form = new QFormLayout(view_body);
  ApplyCompactForm(view_form);

  auto* background_row = new QWidget(view_body);
  auto* background_layout = new QHBoxLayout(background_row);
  background_layout->setContentsMargins(0, 0, 0, 0);
  background_layout->setSpacing(8);

  background_preview_ = new QLabel(background_row);
  background_preview_->setFixedSize(42, 28);
  background_preview_->setFrameShape(QFrame::StyledPanel);
  background_layout->addWidget(background_preview_);

  background_button_ = new QPushButton(tr("Pick color…"), background_row);
  background_button_->setCursor(Qt::PointingHandCursor);
  background_layout->addWidget(background_button_);

  background_r_spin_ = MakeRgbSpinBox(48, background_row);
  background_g_spin_ = MakeRgbSpinBox(48, background_row);
  background_b_spin_ = MakeRgbSpinBox(48, background_row);
  background_layout->addWidget(new QLabel(tr("R"), background_row));
  background_layout->addWidget(background_r_spin_);
  background_layout->addWidget(new QLabel(tr("G"), background_row));
  background_layout->addWidget(background_g_spin_);
  background_layout->addWidget(new QLabel(tr("B"), background_row));
  background_layout->addWidget(background_b_spin_);
  background_layout->addStretch();
  view_form->addRow(tr("Background"), background_row);

  auto* frame_rate_row = new QWidget(view_body);
  auto* frame_rate_layout = new QHBoxLayout(frame_rate_row);
  frame_rate_layout->setContentsMargins(0, 0, 0, 0);
  frame_rate_layout->setSpacing(8);
  frame_rate_spin_ = new QSpinBox(frame_rate_row);
  frame_rate_spin_->setRange(1, 120);
  frame_rate_spin_->setSuffix(tr(" fps"));
  frame_rate_spin_->setFixedWidth(88);
  frame_rate_slider_ = new QSlider(Qt::Horizontal, frame_rate_row);
  frame_rate_slider_->setRange(1, 120);
  frame_rate_layout->addWidget(frame_rate_spin_);
  frame_rate_layout->addWidget(frame_rate_slider_, 1);
  view_form->addRow(tr("Target frame rate"), frame_rate_row);

  show_grid_check_ = new QCheckBox(tr("Show reference grid in 3D view"), view_body);
  view_form->addRow(QString(), show_grid_check_);
  body->addWidget(
      MakeCollapsibleSection(content, tr("3D View"), view_body, true));

  auto* render_body = new QWidget(content);
  auto* render_form = new QFormLayout(render_body);
  ApplyCompactForm(render_form);

  render_backend_combo_ = new QComboBox(render_body);
  render_backend_combo_->addItem(QStringLiteral("OpenGL"), QStringLiteral("OpenGL"));
#ifdef AUTOVIZ_USE_OGRE
  render_backend_combo_->addItem(QStringLiteral("Ogre"), QStringLiteral("Ogre"));
#endif
  render_form->addRow(tr("Render backend"), render_backend_combo_);
  body->addWidget(
      MakeCollapsibleSection(content, tr("Rendering"), render_body, true));

  auto* layout_body = new QWidget(content);
  auto* layout_form = new QFormLayout(layout_body);
  ApplyCompactForm(layout_form);

  show_left_sidebar_check_ =
      new QCheckBox(tr("Show left sidebar on startup"), layout_body);
  show_right_sidebar_check_ =
      new QCheckBox(tr("Show right sidebar on startup"), layout_body);
  show_panel_settings_check_ =
      new QCheckBox(tr("Show panel settings sidebar by default"), layout_body);
  start_maximized_check_ =
      new QCheckBox(tr("Start with maximized main window"), layout_body);
  start_maximized_check_->setChecked(true);
  layout_form->addRow(QString(), show_left_sidebar_check_);
  layout_form->addRow(QString(), show_right_sidebar_check_);
  layout_form->addRow(QString(), show_panel_settings_check_);
  layout_form->addRow(QString(), start_maximized_check_);
  body->addWidget(
      MakeCollapsibleSection(content, tr("Layout"), layout_body, true));

  auto* time_body = new QWidget(content);
  auto* time_form = new QFormLayout(time_body);
  ApplyCompactForm(time_form);

  time_sync_combo_ = new QComboBox(time_body);
  time_sync_combo_->addItem(tr("Off"), static_cast<int>(common::TimeSyncMode::kOff));
  time_sync_combo_->addItem(tr("Exact"),
                            static_cast<int>(common::TimeSyncMode::kExact));
  time_sync_combo_->addItem(tr("Approximate"),
                            static_cast<int>(common::TimeSyncMode::kApproximate));
  time_form->addRow(tr("Time sync mode"), time_sync_combo_);

  time_paused_check_ = new QCheckBox(tr("Start playback paused"), time_body);
  time_form->addRow(QString(), time_paused_check_);
  body->addWidget(
      MakeCollapsibleSection(content, tr("Playback"), time_body, false));

  body->addStretch();
  scroll->setWidget(content);
  root->addWidget(scroll, 1);

  auto* footer = new QFrame(this);
  ApplyPanelFooterChrome(footer);
  auto* footer_layout = new QHBoxLayout(footer);
  footer_layout->setContentsMargins(12, 8, 12, 8);
  auto* footer_hint = new QLabel(
      tr("Changes apply immediately after you click OK and are saved with the session."),
      footer);
  footer_hint->setWordWrap(true);
  StyleHintLabel(footer_hint);
  footer_layout->addWidget(footer_hint, 1);

  auto* buttons =
      new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                           footer);
  connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  footer_layout->addWidget(buttons);
  root->addWidget(footer);

  connect(background_button_, &QPushButton::clicked, this,
          &AppSettingsDialog::pickBackgroundColor);
  connect(frame_rate_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
          frame_rate_slider_, &QSlider::setValue);
  connect(frame_rate_slider_, &QSlider::valueChanged, frame_rate_spin_,
          &QSpinBox::setValue);
  const auto sync_rgb = [this]() {
    syncBackgroundUiFromColor(
        QColor(background_r_spin_->value(), background_g_spin_->value(),
               background_b_spin_->value()));
  };
  connect(background_r_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          sync_rgb);
  connect(background_g_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          sync_rgb);
  connect(background_b_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          sync_rgb);

  if (manager_ != nullptr) {
    populateFrameList();
    AddComboItemIfMissing(fixed_frame_combo_,
                          QString::fromStdString(manager_->fixedFrame()));
    fixed_frame_combo_->setCurrentText(QString::fromStdString(manager_->fixedFrame()));

    const QString transformer_id =
        QString::fromStdString(manager_->transformationManager().currentTransformerId());
    const int transformer_index = transformer_combo_->findData(transformer_id);
    if (transformer_index >= 0) {
      transformer_combo_->setCurrentIndex(transformer_index);
    }

    const QString view_name = QString::fromStdString(manager_->viewControllerName());
    const int view_index = view_controller_combo_->findData(view_name);
    if (view_index >= 0) {
      view_controller_combo_->setCurrentIndex(view_index);
    }

    const QString backend = QString::fromStdString(manager_->renderBackendName());
    const int backend_index = render_backend_combo_->findData(backend);
    if (backend_index >= 0) {
      render_backend_combo_->setCurrentIndex(backend_index);
    }

    frame_rate_spin_->setValue(manager_->targetFrameRate());
    frame_rate_slider_->setValue(manager_->targetFrameRate());
    syncBackgroundUiFromColor(common::ParseColorProperty(
        manager_->backgroundColor(), QColor(48, 48, 48)));
    show_grid_check_->setChecked(manager_->showGrid());

    const int sync_index =
        time_sync_combo_->findData(static_cast<int>(manager_->timeSyncMode()));
    if (sync_index >= 0) {
      time_sync_combo_->setCurrentIndex(sync_index);
    }
    time_paused_check_->setChecked(manager_->timePaused());

    show_left_sidebar_check_->setChecked(!manager_->hideLeftDock());
    show_right_sidebar_check_->setChecked(!manager_->hideRightDock());
    show_panel_settings_check_->setChecked(manager_->plotSettingsVisible());
  }

  const QString language_value = ui_prefs.language_code;
  const int language_index =
      language_combo_->findData(language_value.isEmpty() ? QString() : language_value);
  if (language_index >= 0) {
    language_combo_->setCurrentIndex(language_index);
  }
  shortcuts_editor_->setShortcuts(ui_prefs.shortcuts);
  if (start_maximized_check_ != nullptr) {
    start_maximized_check_->setChecked(ui_prefs.start_maximized);
  }
}

void AppSettingsDialog::populateFrameList() {
  if (fixed_frame_combo_ == nullptr) {
    return;
  }
  const QString current = fixed_frame_combo_->currentText();
  fixed_frame_combo_->clear();
  const QStringList frames = CollectTfFrames(manager_);
  fixed_frame_combo_->addItems(frames);
  if (!current.isEmpty()) {
    AddComboItemIfMissing(fixed_frame_combo_, current);
    fixed_frame_combo_->setCurrentText(current);
  }
}

void AppSettingsDialog::syncBackgroundUiFromColor(const QColor& color) {
  const QColor resolved = color.isValid() ? color : QColor(48, 48, 48);
  background_r_spin_->blockSignals(true);
  background_g_spin_->blockSignals(true);
  background_b_spin_->blockSignals(true);
  background_r_spin_->setValue(resolved.red());
  background_g_spin_->setValue(resolved.green());
  background_b_spin_->setValue(resolved.blue());
  background_r_spin_->blockSignals(false);
  background_g_spin_->blockSignals(false);
  background_b_spin_->blockSignals(false);

  background_preview_->setStyleSheet(
      QStringLiteral("background: rgb(%1,%2,%3); border: 1px solid palette(mid);"
                     " border-radius: 4px;")
          .arg(resolved.red())
          .arg(resolved.green())
          .arg(resolved.blue()));
  UpdateColorButton(background_button_, resolved);
}

QColor AppSettingsDialog::backgroundColorFromUi() const {
  return QColor(background_r_spin_->value(), background_g_spin_->value(),
                background_b_spin_->value());
}

void AppSettingsDialog::pickBackgroundColor() {
  const QColor chosen =
      QColorDialog::getColor(backgroundColorFromUi(), this, tr("Background color"));
  if (!chosen.isValid()) {
    return;
  }
  syncBackgroundUiFromColor(chosen);
}

AppSettingsResult AppSettingsDialog::resultValues() const {
  AppSettingsResult result;

  if (fixed_frame_combo_ != nullptr && manager_ != nullptr) {
    result.fixed_frame = fixed_frame_combo_->currentText().trimmed().toStdString();
    result.transformer_id =
        transformer_combo_->currentData().toString().trimmed().toStdString();
    result.view_controller =
        view_controller_combo_->currentData().toString().trimmed().toStdString();
    result.render_backend =
        render_backend_combo_->currentData().toString().trimmed().toStdString();
    result.background_color =
        common::FormatColorProperty(backgroundColorFromUi());
    result.frame_rate = frame_rate_spin_->value();
    result.show_grid = show_grid_check_->isChecked();
    result.time_sync_mode =
        static_cast<common::TimeSyncMode>(time_sync_combo_->currentData().toInt());
    result.time_paused = time_paused_check_->isChecked();
    result.hide_left_dock = !show_left_sidebar_check_->isChecked();
    result.hide_right_dock = !show_right_sidebar_check_->isChecked();
    result.plot_settings_visible = show_panel_settings_check_->isChecked();
  }

  result.language_code = language_combo_->currentData().toString();
  result.start_maximized =
      start_maximized_check_ != nullptr && start_maximized_check_->isChecked();
  if (shortcuts_editor_ != nullptr) {
    result.shortcuts = shortcuts_editor_->shortcuts();
  }
  return result;
}

}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/app/settings_dialog.hpp"

#include <functional>

#include <QAbstractItemView>
#include <QApplication>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFont>
#include <QFrame>
#include <QHash>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QSpinBox>
#include <QStackedWidget>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/common/display_property.hpp"
#include "autoviz/common/transformation/transformation_manager.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/common/widget/input/segmented_control.hpp"
#include "autoviz/transform/buffer.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/app/preferences.hpp"
#include "autoviz/ui/app/shortcuts_editor.hpp"
#include "autoviz/ui/theme/application.hpp"
#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

constexpr int kSidebarW = 200;
constexpr int kRowH = 40;

struct AccentSpec {
  QColor color;
  const char* name;
};

const AccentSpec kAccentPalette[] = {
    {QColor(0x5B, 0x7A, 0x1E), QT_TR_NOOP("Olive")},
    {QColor(0x00, 0x7A, 0xFF), QT_TR_NOOP("Blue")},
    {QColor(0x72, 0x39, 0xE6), QT_TR_NOOP("Violet")},
    {QColor(0xEC, 0x48, 0x99), QT_TR_NOOP("Pink")},
    {QColor(0xEA, 0x58, 0x0C), QT_TR_NOOP("Orange")},
    {QColor(0x14, 0xB8, 0xA6), QT_TR_NOOP("Teal")},
};

QStringList CollectTfFrames(common::VisualizationManager* manager) {
  QStringList frames;
  if (manager == nullptr || manager->tfBuffer() == nullptr) {
    return frames;
  }
  for (const transform::TfFrameStats& stats :
       manager->tfBuffer()->frameStats()) {
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

QHash<QString, QString> AppSettingsTokens() { return style::tokens(); }

QFrame* MakeGroup(QWidget* parent) {
  auto* group = new QFrame(parent);
  group->setObjectName(QStringLiteral("AutovizSettingsGroup"));
  auto* layout = new QVBoxLayout(group);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  return group;
}

QFrame* MakeHairline(QWidget* parent) {
  auto* line = new QFrame(parent);
  line->setFixedHeight(1);
  line->setStyleSheet(style::mark(style::Mark::Rule));
  return line;
}

QWidget* MakeRow(QWidget* parent, const QString& label, QWidget* field,
                 bool stretch_field = true) {
  auto* row = new QWidget(parent);
  row->setMinimumHeight(kRowH);
  auto* layout = new QHBoxLayout(row);
  layout->setContentsMargins(16, 10, 16, 10);
  layout->setSpacing(12);

  auto* text = new QLabel(label, row);
  text->setObjectName(QStringLiteral("AutovizSettingsRowLabel"));
  layout->addWidget(text, 0, Qt::AlignVCenter);
  layout->addStretch(1);
  if (field != nullptr) {
    field->setParent(row);
    layout->addWidget(field, stretch_field ? 0 : 0, Qt::AlignVCenter);
  }
  return row;
}

QWidget* MakeCheckRow(QWidget* parent, QCheckBox* check) {
  auto* row = new QWidget(parent);
  row->setMinimumHeight(kRowH);
  auto* layout = new QHBoxLayout(row);
  layout->setContentsMargins(16, 10, 16, 10);
  layout->setSpacing(0);
  check->setParent(row);
  layout->addWidget(check);
  layout->addStretch(1);
  return row;
}

QScrollArea* WrapPage(QWidget* page_body) {
  auto* scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setStyleSheet(style::sheet(QStringLiteral("chrome/layout")));
  page_body->setStyleSheet(style::mark(style::Mark::Clear));
  scroll->setWidget(page_body);
  return scroll;
}

QWidget* MakePageShell(QWidget* parent, QWidget* group_content) {
  auto* page = new QWidget(parent);
  auto* layout = new QVBoxLayout(page);
  layout->setContentsMargins(28, 8, 28, 24);
  layout->setSpacing(16);
  layout->addWidget(group_content);
  layout->addStretch(1);
  return page;
}

QString AccentDisplayName(const QString& accent_hex) {
  if (accent_hex.isEmpty()) {
    return QObject::tr("Default");
  }
  const QColor color(accent_hex);
  if (!color.isValid()) {
    return QObject::tr("Custom");
  }
  for (const AccentSpec& spec : kAccentPalette) {
    if (spec.color.name(QColor::HexRgb).compare(color.name(QColor::HexRgb),
                                                Qt::CaseInsensitive) == 0) {
      return QObject::tr(spec.name);
    }
  }
  return QObject::tr("%1 %2")
      .arg(QObject::tr("Custom"), color.name(QColor::HexRgb).toUpper());
}

class AccentSwatch : public QToolButton {
 public:
  AccentSwatch(const QColor& color, const QString& name, QWidget* parent = nullptr)
      : QToolButton(parent), color_(color), name_(name) {
    setObjectName(QStringLiteral("AutovizAccentSwatch"));
    setCursor(Qt::PointingHandCursor);
    setFixedSize(28, 28);
    setFocusPolicy(Qt::StrongFocus);
    setToolTip(name_);
  }

  QColor color() const { return color_; }
  QString accentName() const { return name_; }

  void setSelected(bool selected) {
    if (selected_ == selected) {
      return;
    }
    selected_ = selected;
    update();
  }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const QRectF outer = QRectF(rect()).adjusted(1.5, 1.5, -1.5, -1.5);
    if (selected_) {
      painter.setPen(QPen(QColor(0x1C, 0x1C, 0x1E), 2.0));
      painter.setBrush(Qt::NoBrush);
      painter.drawEllipse(outer.adjusted(-1.0, -1.0, 1.0, 1.0));
    }
    painter.setPen(QPen(QColor(0, 0, 0, 30), 1.0));
    painter.setBrush(color_);
    painter.drawEllipse(outer.adjusted(2.0, 2.0, -2.0, -2.0));
  }

 private:
  QColor color_;
  QString name_;
  bool selected_ = false;
};

}  // namespace

AppSettingsDialog::AppSettingsDialog(common::VisualizationManager* manager,
                                     QWidget* parent)
    : manager_(manager), QDialog(parent) {
  setWindowTitle(tr("Settings"));
  setModal(true);
  resize(760, 580);
  setMinimumSize(680, 500);
  setObjectName(QStringLiteral("AutovizAppSettingsDialog"));
  setStyleSheet(
      style::sheet(QStringLiteral("app_settings"), AppSettingsTokens()));

  const AppUiPreferences ui_prefs = LoadAppUiPreferences();
  theme_mode_ = ui_prefs.theme_mode.isEmpty() ? QStringLiteral("light")
                                              : ui_prefs.theme_mode;
  accent_color_ = ui_prefs.accent_color;
  preview_restore_theme_ = theme_mode_;
  preview_restore_accent_ = accent_color_;

  auto* root = new QHBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  sidebar_ = new QListWidget(this);
  sidebar_->setObjectName(QStringLiteral("AutovizSettingsSidebar"));
  sidebar_->setFixedWidth(kSidebarW);
  sidebar_->setIconSize(QSize(18, 18));
  sidebar_->setSpacing(3);
  sidebar_->setUniformItemSizes(true);
  sidebar_->setSelectionMode(QAbstractItemView::SingleSelection);
  sidebar_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  sidebar_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  struct Category {
    const char* title;
    const char* icon_id;
  };
  // Concat order: General → Appearance → …
  const Category categories[] = {
      {QT_TR_NOOP("General"), "app.settings"},
      {QT_TR_NOOP("Appearance"), "file.image"},
      {QT_TR_NOOP("3D View"), "panels.fullscreen"},
      {QT_TR_NOOP("Layout"), "file.reset_layout"},
      {QT_TR_NOOP("Shortcuts"), "panels.tools"},
      {QT_TR_NOOP("Playback"), "file.recent"},
  };
  for (const Category& cat : categories) {
    auto* item = new QListWidgetItem(tr(cat.title));
    item->setIcon(IconLoader::menuIcon(QString::fromLatin1(cat.icon_id)));
    item->setSizeHint(QSize(kSidebarW - 16, 38));
    sidebar_->addItem(item);
  }

  auto* detail = new QWidget(this);
  detail->setObjectName(QStringLiteral("AutovizSettingsDetail"));
  auto* detail_layout = new QVBoxLayout(detail);
  detail_layout->setContentsMargins(0, 0, 0, 0);
  detail_layout->setSpacing(0);

  auto* header = new QWidget(detail);
  auto* header_layout = new QVBoxLayout(header);
  header_layout->setContentsMargins(28, 22, 28, 10);
  header_layout->setSpacing(0);
  page_title_ = new QLabel(header);
  page_title_->setObjectName(QStringLiteral("AutovizSettingsPageTitle"));
  header_layout->addWidget(page_title_);
  detail_layout->addWidget(header);

  pages_ = new QStackedWidget(detail);

  // Page 0: General (language + TF / backend)
  {
    auto* group = MakeGroup(pages_);
    auto* gl = qobject_cast<QVBoxLayout*>(group->layout());

    language_combo_ = new QComboBox;
    language_combo_->setMinimumWidth(220);
    language_combo_->addItem(tr("Follow system"), QString());
    language_combo_->addItem(tr("English"), QStringLiteral("en"));
    language_combo_->addItem(tr("简体中文"), QStringLiteral("zh_CN"));
    gl->addWidget(MakeRow(group, tr("Language"), language_combo_));
    gl->addWidget(MakeHairline(group));

    fixed_frame_combo_ = new QComboBox;
    fixed_frame_combo_->setEditable(true);
    fixed_frame_combo_->setInsertPolicy(QComboBox::NoInsert);
    fixed_frame_combo_->setMinimumWidth(220);
    gl->addWidget(MakeRow(group, tr("Fixed frame"), fixed_frame_combo_));
    gl->addWidget(MakeHairline(group));

    transformer_combo_ = new QComboBox;
    transformer_combo_->setMinimumWidth(220);
    if (manager_ != nullptr) {
      for (const common::PluginInfo& info :
           manager_->transformationManager().availableTransformers()) {
        transformer_combo_->addItem(QString::fromStdString(info.name),
                                     QString::fromStdString(info.class_id));
      }
    }
    gl->addWidget(MakeRow(group, tr("TF transformer"), transformer_combo_));
    gl->addWidget(MakeHairline(group));

    render_backend_combo_ = new QComboBox;
    render_backend_combo_->setMinimumWidth(220);
    render_backend_combo_->addItem(QStringLiteral("Ogre"),
                                   QStringLiteral("Ogre"));
    render_backend_combo_->setEnabled(false);
    render_backend_combo_->setToolTip(
        tr("Autoviz uses the Ogre 1.x viewport only."));
    gl->addWidget(MakeRow(group, tr("Render backend"), render_backend_combo_));

    pages_->addWidget(WrapPage(MakePageShell(pages_, group)));
  }

  // Page 1: Appearance (Concat-style theme + accent)
  {
    auto* group = MakeGroup(pages_);
    auto* gl = qobject_cast<QVBoxLayout*>(group->layout());

    theme_segments_ = new widget::SegmentedControl(group);
    theme_segments_->SetTone(style::Tone::Frost);
    theme_segments_->setMinimumWidth(280);
    theme_segments_->setFixedHeight(34);
    theme_segments_->AddSegment(tr("Dark"), QStringLiteral("dark"));
    theme_segments_->AddSegment(tr("Light"), QStringLiteral("light"));
    theme_segments_->AddSegment(tr("Liquid"), QStringLiteral("liquid"));
    gl->addWidget(MakeRow(group, tr("Theme"), theme_segments_, false));
    gl->addWidget(MakeHairline(group));

    accent_host_ = new QWidget(group);
    auto* accent_layout = new QHBoxLayout(accent_host_);
    accent_layout->setContentsMargins(0, 0, 0, 0);
    accent_layout->setSpacing(10);
    for (const AccentSpec& spec : kAccentPalette) {
      auto* swatch =
          new AccentSwatch(spec.color, tr(spec.name), accent_host_);
      connect(swatch, &QToolButton::clicked, this, [this, color = spec.color]() {
        setAccentColor(color.name(QColor::HexRgb).toUpper());
      });
      accent_layout->addWidget(swatch, 0, Qt::AlignVCenter);
    }
    auto* custom = new QToolButton(accent_host_);
    custom->setObjectName(QStringLiteral("AutovizAccentCustom"));
    custom->setText(QStringLiteral("+"));
    custom->setCursor(Qt::PointingHandCursor);
    custom->setFixedSize(28, 28);
    custom->setToolTip(tr("Custom accent color"));
    connect(custom, &QToolButton::clicked, this, [this]() {
      const QColor seed =
          accent_color_.isEmpty() ? glass::Shell().accent : QColor(accent_color_);
      const QColor chosen =
          QColorDialog::getColor(seed, this, tr("Accent color"));
      if (chosen.isValid()) {
        setAccentColor(chosen.name(QColor::HexRgb).toUpper());
      }
    });
    accent_layout->addWidget(custom, 0, Qt::AlignVCenter);
    accent_name_label_ = new QLabel(accent_host_);
    accent_name_label_->setObjectName(
        QStringLiteral("AutovizSettingsAccentName"));
    accent_layout->addWidget(accent_name_label_, 0, Qt::AlignVCenter);
    accent_layout->addStretch(1);
    gl->addWidget(MakeRow(group, tr("Accent color"), accent_host_, false));

    pages_->addWidget(WrapPage(MakePageShell(pages_, group)));
  }

  // Page 2: 3D View
  {
    auto* group = MakeGroup(pages_);
    auto* gl = qobject_cast<QVBoxLayout*>(group->layout());

    auto* bg_field = new QWidget;
    auto* bg_layout = new QHBoxLayout(bg_field);
    bg_layout->setContentsMargins(0, 0, 0, 0);
    bg_layout->setSpacing(8);
    background_preview_ = new QLabel(bg_field);
    background_preview_->setFixedSize(22, 22);
    background_preview_->setStyleSheet(style::mark(style::Mark::Preview));
    background_button_ = new QPushButton(tr("Pick…"), bg_field);
    background_button_->setCursor(Qt::PointingHandCursor);
    background_button_->setFixedWidth(92);
    auto* reset_bg = new QPushButton(tr("Default"), bg_field);
    reset_bg->setCursor(Qt::PointingHandCursor);
    bg_layout->addWidget(background_preview_);
    bg_layout->addWidget(background_button_);
    bg_layout->addWidget(reset_bg);
    connect(reset_bg, &QPushButton::clicked, this, [this]() {
      syncBackgroundUiFromColor(AppThemeSuggestedViewportBackground());
    });
    gl->addWidget(MakeRow(group, tr("Background"), bg_field, false));
    gl->addWidget(MakeHairline(group));

    background_r_spin_ = new QSpinBox(group);
    background_g_spin_ = new QSpinBox(group);
    background_b_spin_ = new QSpinBox(group);
    for (QSpinBox* spin :
         {background_r_spin_, background_g_spin_, background_b_spin_}) {
      spin->setRange(0, 255);
      spin->hide();
    }

    auto* fps_field = new QWidget;
    auto* fps_layout = new QHBoxLayout(fps_field);
    fps_layout->setContentsMargins(0, 0, 0, 0);
    fps_layout->setSpacing(10);
    frame_rate_spin_ = new QSpinBox(fps_field);
    frame_rate_spin_->setRange(1, 120);
    frame_rate_spin_->setSuffix(tr(" fps"));
    frame_rate_spin_->setFixedWidth(84);
    frame_rate_slider_ = new QSlider(Qt::Horizontal, fps_field);
    frame_rate_slider_->setRange(1, 120);
    frame_rate_slider_->setFixedWidth(140);
    fps_layout->addWidget(frame_rate_slider_);
    fps_layout->addWidget(frame_rate_spin_);
    gl->addWidget(MakeRow(group, tr("Frame rate"), fps_field, false));

    pages_->addWidget(WrapPage(MakePageShell(pages_, group)));
  }

  // Page 3: Layout
  {
    auto* group = MakeGroup(pages_);
    auto* gl = qobject_cast<QVBoxLayout*>(group->layout());

    show_left_sidebar_check_ =
        new QCheckBox(tr("Show left sidebar on startup"));
    show_right_sidebar_check_ =
        new QCheckBox(tr("Show right sidebar on startup"));
    show_panel_settings_check_ =
        new QCheckBox(tr("Show panel settings sidebar by default"));
    start_maximized_check_ =
        new QCheckBox(tr("Start with maximized main window"));
    start_maximized_check_->setChecked(true);

    gl->addWidget(MakeCheckRow(group, show_left_sidebar_check_));
    gl->addWidget(MakeHairline(group));
    gl->addWidget(MakeCheckRow(group, show_right_sidebar_check_));
    gl->addWidget(MakeHairline(group));
    gl->addWidget(MakeCheckRow(group, show_panel_settings_check_));
    gl->addWidget(MakeHairline(group));
    gl->addWidget(MakeCheckRow(group, start_maximized_check_));

    pages_->addWidget(WrapPage(MakePageShell(pages_, group)));
  }

  // Page 4: Shortcuts
  {
    auto* page = new QWidget(pages_);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(28, 8, 28, 24);
    layout->setSpacing(12);

    auto* group = MakeGroup(page);
    auto* gl = qobject_cast<QVBoxLayout*>(group->layout());
    gl->setContentsMargins(12, 12, 12, 12);
    gl->setSpacing(10);
    shortcuts_editor_ = new ShortcutsEditorWidget(group);
    gl->addWidget(shortcuts_editor_);
    auto* reset_shortcuts = new QPushButton(tr("Reset to Defaults"), group);
    reset_shortcuts->setCursor(Qt::PointingHandCursor);
    gl->addWidget(reset_shortcuts, 0, Qt::AlignLeft);
    connect(reset_shortcuts, &QPushButton::clicked, shortcuts_editor_,
            &ShortcutsEditorWidget::resetToDefaults);

    layout->addWidget(group);
    layout->addStretch(1);
    pages_->addWidget(WrapPage(page));
  }

  // Page 5: Playback
  {
    auto* group = MakeGroup(pages_);
    auto* gl = qobject_cast<QVBoxLayout*>(group->layout());

    time_sync_combo_ = new QComboBox;
    time_sync_combo_->setMinimumWidth(180);
    time_sync_combo_->addItem(tr("Off"),
                              static_cast<int>(common::TimeSyncMode::kOff));
    time_sync_combo_->addItem(tr("Exact"),
                              static_cast<int>(common::TimeSyncMode::kExact));
    time_sync_combo_->addItem(
        tr("Approximate"),
        static_cast<int>(common::TimeSyncMode::kApproximate));
    gl->addWidget(MakeRow(group, tr("Time sync"), time_sync_combo_));
    gl->addWidget(MakeHairline(group));

    time_paused_check_ = new QCheckBox(tr("Start playback paused"));
    gl->addWidget(MakeCheckRow(group, time_paused_check_));

    pages_->addWidget(WrapPage(MakePageShell(pages_, group)));
  }

  detail_layout->addWidget(pages_, 1);

  auto* footer = new QFrame(detail);
  footer->setObjectName(QStringLiteral("AutovizSettingsFooter"));
  auto* footer_layout = new QHBoxLayout(footer);
  footer_layout->setContentsMargins(20, 10, 20, 12);
  auto* hint = new QLabel(tr("Changes are saved with the session."), footer);
  hint->setObjectName(QStringLiteral("AutovizSettingsHint"));
  footer_layout->addWidget(hint, 1);
  auto* buttons = new QDialogButtonBox(
      QDialogButtonBox::Cancel | QDialogButtonBox::Ok, footer);
  if (QPushButton* ok = buttons->button(QDialogButtonBox::Ok)) {
    ok->setText(tr("Done"));
    ok->setDefault(true);
  }
  if (QPushButton* cancel = buttons->button(QDialogButtonBox::Cancel)) {
    cancel->setText(tr("Cancel"));
  }
  connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  footer_layout->addWidget(buttons);
  detail_layout->addWidget(footer);

  root->addWidget(sidebar_);
  root->addWidget(detail, 1);

  connect(sidebar_, &QListWidget::currentRowChanged, this,
          &AppSettingsDialog::showCategory);
  connect(background_button_, &QPushButton::clicked, this,
          &AppSettingsDialog::pickBackgroundColor);
  connect(frame_rate_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
          frame_rate_slider_, &QSlider::setValue);
  connect(frame_rate_slider_, &QSlider::valueChanged, frame_rate_spin_,
          &QSpinBox::setValue);
  connect(theme_segments_, &widget::SegmentedControl::CurrentIndexChanged, this,
          [this](int) {
            if (theme_segments_ == nullptr) {
              return;
            }
            setThemeMode(theme_segments_->current_data().toString());
          });
  connect(this, &QDialog::rejected, this, [this]() {
    if (QApplication* app =
            qobject_cast<QApplication*>(QApplication::instance())) {
      ApplyUiTheme(preview_restore_theme_, preview_restore_accent_, *app);
    }
  });

  if (manager_ != nullptr) {
    populateFrameList();
    AddComboItemIfMissing(fixed_frame_combo_,
                          QString::fromStdString(manager_->fixedFrame()));
    fixed_frame_combo_->setCurrentText(
        QString::fromStdString(manager_->fixedFrame()));

    const QString transformer_id = QString::fromStdString(
        manager_->transformationManager().currentTransformerId());
    const int transformer_index = transformer_combo_->findData(transformer_id);
    if (transformer_index >= 0) {
      transformer_combo_->setCurrentIndex(transformer_index);
    }

    const QString backend =
        QString::fromStdString(manager_->renderBackendName());
    const int backend_index = render_backend_combo_->findData(backend);
    if (backend_index >= 0) {
      render_backend_combo_->setCurrentIndex(backend_index);
    }

    frame_rate_spin_->setValue(manager_->targetFrameRate());
    frame_rate_slider_->setValue(manager_->targetFrameRate());
    syncBackgroundUiFromColor(common::ParseColorProperty(
        manager_->backgroundColor(), QColor(48, 48, 48)));

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

  const int language_index = language_combo_->findData(
      ui_prefs.language_code.isEmpty() ? QString() : ui_prefs.language_code);
  if (language_index >= 0) {
    language_combo_->setCurrentIndex(language_index);
  }

  // Map "system" → Light/Dark based on resolved preference for the segment UI.
  QString segment_mode = theme_mode_;
  if (segment_mode == QLatin1String("system")) {
    segment_mode =
        glass::ShellIsDark() ? QStringLiteral("dark") : QStringLiteral("light");
  }
  int theme_index = 1;  // Light
  for (int i = 0; i < theme_segments_->count(); ++i) {
    if (theme_segments_->segment_data(i).toString() == segment_mode) {
      theme_index = i;
      break;
    }
  }
  theme_segments_->blockSignals(true);
  theme_segments_->SetCurrentIndex(theme_index);
  theme_segments_->blockSignals(false);
  theme_mode_ = theme_segments_->current_data().toString();
  syncAccentSwatchSelection();
  if (accent_name_label_ != nullptr) {
    accent_name_label_->setText(AccentDisplayName(accent_color_));
  }

  shortcuts_editor_->setShortcuts(ui_prefs.shortcuts);
  start_maximized_check_->setChecked(ui_prefs.start_maximized);

  sidebar_->setCurrentRow(0);
  showCategory(0);
}

void AppSettingsDialog::showCategory(int index) {
  if (pages_ == nullptr || page_title_ == nullptr || sidebar_ == nullptr) {
    return;
  }
  if (index < 0 || index >= pages_->count()) {
    return;
  }
  pages_->setCurrentIndex(index);
  if (QListWidgetItem* item = sidebar_->item(index)) {
    page_title_->setText(item->text());
  }
}

void AppSettingsDialog::setThemeMode(const QString& theme_mode) {
  if (theme_mode.isEmpty()) {
    return;
  }
  theme_mode_ = theme_mode;
  applyLiveThemePreview();
}

void AppSettingsDialog::setAccentColor(const QString& accent_hex) {
  accent_color_ = accent_hex;
  syncAccentSwatchSelection();
  if (accent_name_label_ != nullptr) {
    accent_name_label_->setText(AccentDisplayName(accent_color_));
  }
  applyLiveThemePreview();
}

void AppSettingsDialog::applyLiveThemePreview() {
  if (QApplication* app =
          qobject_cast<QApplication*>(QApplication::instance())) {
    ApplyUiTheme(theme_mode_, accent_color_, *app);
    setStyleSheet(
        style::sheet(QStringLiteral("app_settings"), AppSettingsTokens()));
  }
}

void AppSettingsDialog::syncAccentSwatchSelection() {
  if (accent_host_ == nullptr) {
    return;
  }
  const QColor selected(accent_color_);
  for (AccentSwatch* swatch : accent_host_->findChildren<AccentSwatch*>()) {
    if (swatch == nullptr) {
      continue;
    }
    const bool match =
        selected.isValid() &&
        swatch->color().name(QColor::HexRgb).compare(
            selected.name(QColor::HexRgb), Qt::CaseInsensitive) == 0;
    swatch->setSelected(match);
  }
}

void AppSettingsDialog::populateFrameList() {
  if (fixed_frame_combo_ == nullptr) {
    return;
  }
  const QString current = fixed_frame_combo_->currentText();
  fixed_frame_combo_->clear();
  fixed_frame_combo_->addItems(CollectTfFrames(manager_));
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

  background_preview_->setStyleSheet(style::mark(
      style::Mark::Fill,
      QStringLiteral("%1,%2,%3")
          .arg(resolved.red())
          .arg(resolved.green())
          .arg(resolved.blue())));
  background_button_->setText(resolved.name(QColor::HexRgb).toUpper());
}

QColor AppSettingsDialog::backgroundColorFromUi() const {
  return QColor(background_r_spin_->value(), background_g_spin_->value(),
                background_b_spin_->value());
}

void AppSettingsDialog::pickBackgroundColor() {
  const QColor chosen = QColorDialog::getColor(backgroundColorFromUi(), this,
                                               tr("Background color"));
  if (!chosen.isValid()) {
    return;
  }
  syncBackgroundUiFromColor(chosen);
}

AppSettingsResult AppSettingsDialog::resultValues() const {
  AppSettingsResult result;
  if (fixed_frame_combo_ != nullptr && manager_ != nullptr) {
    result.fixed_frame =
        fixed_frame_combo_->currentText().trimmed().toStdString();
    result.transformer_id =
        transformer_combo_->currentData().toString().trimmed().toStdString();
    result.render_backend =
        render_backend_combo_->currentData().toString().trimmed().toStdString();
    result.background_color =
        common::FormatColorProperty(backgroundColorFromUi());
    result.frame_rate = frame_rate_spin_->value();
    result.time_sync_mode = static_cast<common::TimeSyncMode>(
        time_sync_combo_->currentData().toInt());
    result.time_paused = time_paused_check_->isChecked();
    result.hide_left_dock = !show_left_sidebar_check_->isChecked();
    result.hide_right_dock = !show_right_sidebar_check_->isChecked();
    result.plot_settings_visible = show_panel_settings_check_->isChecked();
  }
  result.language_code = language_combo_->currentData().toString();
  result.theme_mode = theme_mode_.isEmpty() ? QStringLiteral("light")
                                            : theme_mode_;
  result.accent_color = accent_color_;
  result.start_maximized = start_maximized_check_->isChecked();
  if (shortcuts_editor_ != nullptr) {
    result.shortcuts = shortcuts_editor_->shortcuts();
  }
  return result;
}

}  // namespace autoviz

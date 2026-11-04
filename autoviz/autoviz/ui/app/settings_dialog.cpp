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
#include <QKeyEvent>
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
#include <QVBoxLayout>

#include "autoviz/common/display_property.hpp"
#include "autoviz/common/transformation/transformation_manager.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/transform/buffer.hpp"
#include "autoviz/ui/app/preferences.hpp"
#include "autoviz/ui/app/shortcuts_editor.hpp"
#include "autoviz/ui/theme/application.hpp"
#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/style.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/theme/panel.hpp"

namespace autoviz {
namespace {

constexpr int kSidebarW = 196;
constexpr int kRowH = 36;

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

QHash<QString, QString> AppSettingsTokens() {
  // Follow Shell tokens (Liquid Glass system blue / Light / Dark) — do not
  // reintroduce a second teal accent that fights the chrome.
  return style::tokens();
}

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
  layout->setContentsMargins(14, 8, 14, 8);
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
  layout->setContentsMargins(14, 8, 14, 8);
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
  scroll->setStyleSheet(
      style::sheet(QStringLiteral("chrome/layout")));
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

/** Miniature chrome mock for Appearance → Theme cards. */
class ThemePreviewCard : public QFrame {
 public:
  struct Palette {
    QColor window;
    QColor bar;
    QColor sidebar;
    QColor content;
    QColor text;
    QColor muted;
    QColor accent;
    QColor rim;
    bool liquid = false;
  };

  ThemePreviewCard(const QString& mode, const QString& title,
                   const Palette& palette, QWidget* parent = nullptr)
      : QFrame(parent), mode_(mode), title_(title), palette_(palette) {
    setObjectName(QStringLiteral("AutovizThemePreviewCard"));
    setCursor(Qt::PointingHandCursor);
    setFixedSize(148, 128);
    setFocusPolicy(Qt::StrongFocus);
  }

  QString mode() const { return mode_; }

  void setSelected(bool selected) {
    if (selected_ == selected) {
      return;
    }
    selected_ = selected;
    update();
  }

  bool isSelected() const { return selected_; }

  std::function<void()> onClicked;

 protected:
  void mousePressEvent(QMouseEvent* event) override {
    if (event != nullptr && event->button() == Qt::LeftButton && onClicked) {
      onClicked();
    }
    QFrame::mousePressEvent(event);
  }

  void keyPressEvent(QKeyEvent* event) override {
    if (event != nullptr &&
        (event->key() == Qt::Key_Return || event->key() == Qt::Key_Space) &&
        onClicked) {
      onClicked();
      return;
    }
    QFrame::keyPressEvent(event);
  }

  void paintEvent(QPaintEvent* event) override {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QRectF outer = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    QPainterPath card;
    card.addRoundedRect(outer, 12.0, 12.0);

    painter.fillPath(card, palette_.window);
    if (palette_.liquid) {
      QLinearGradient wash(outer.topRight(), outer.bottomLeft());
      wash.setColorAt(0.0, QColor(180, 220, 255, 40));
      wash.setColorAt(0.55, QColor(255, 255, 255, 0));
      wash.setColorAt(1.0, QColor(255, 200, 230, 24));
      painter.fillPath(card, wash);
    }

    // Mini window chrome inside the card.
    const QRectF stage = outer.adjusted(10.0, 10.0, -10.0, -28.0);
    QPainterPath stage_path;
    stage_path.addRoundedRect(stage, 8.0, 8.0);
    painter.setClipPath(stage_path);

    painter.fillRect(stage, palette_.content);
    const QRectF bar(stage.left(), stage.top(), stage.width(), 12.0);
    painter.fillRect(bar, palette_.bar);
    if (palette_.liquid) {
      QLinearGradient sheen(bar.topLeft(), bar.bottomLeft());
      sheen.setColorAt(0.0, QColor(255, 255, 255, 160));
      sheen.setColorAt(1.0, QColor(255, 255, 255, 0));
      painter.fillRect(bar, sheen);
    }
    painter.fillRect(QRectF(stage.left(), bar.bottom(), 28.0,
                            stage.height() - bar.height()),
                     palette_.sidebar);
    painter.setBrush(palette_.accent);
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(
        QRectF(stage.left() + 36.0, stage.top() + 28.0, 42.0, 10.0), 4.0, 4.0);
    painter.setBrush(palette_.muted);
    painter.drawRoundedRect(
        QRectF(stage.left() + 36.0, stage.top() + 44.0, 58.0, 6.0), 3.0, 3.0);
    painter.drawRoundedRect(
        QRectF(stage.left() + 36.0, stage.top() + 54.0, 48.0, 6.0), 3.0, 3.0);

    painter.setClipping(false);

    // Title under the mock.
    QFont font = painter.font();
    font.setPointSizeF(10.5);
    font.setWeight(selected_ ? QFont::DemiBold : QFont::Medium);
    painter.setFont(font);
    painter.setPen(palette_.text);
    painter.drawText(QRectF(outer.left(), outer.bottom() - 22.0, outer.width(),
                            18.0),
                     Qt::AlignHCenter | Qt::AlignVCenter, title_);

    // Selection / idle rim.
    if (selected_) {
      painter.setPen(QPen(palette_.accent, 2.0));
    } else {
      painter.setPen(QPen(palette_.rim, 1.0));
    }
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(card);

    if (selected_ && palette_.liquid) {
      painter.setPen(QPen(QColor(255, 90, 120, 50), 1.0));
      painter.drawRoundedRect(outer.adjusted(2.0, 2.0, -1.0, -1.0), 11.0, 11.0);
      painter.setPen(QPen(QColor(80, 180, 255, 55), 1.0));
      painter.drawRoundedRect(outer.adjusted(1.0, 1.0, -2.0, -2.0), 11.0, 11.0);
    }
  }

 private:
  QString mode_;
  QString title_;
  Palette palette_;
  bool selected_ = false;
};

ThemePreviewCard::Palette LightPreviewPalette() {
  return {QColor(0xF2, 0xF2, 0xF7), QColor(0xFC, 0xFD, 0xFD),
          QColor(0xE8, 0xE8, 0xED), QColor(0xFF, 0xFF, 0xFF),
          QColor(0x1C, 0x1C, 0x1E), QColor(0xC6, 0xC6, 0xC8),
          QColor(0x00, 0x7A, 0xFF), QColor(0xC6, 0xC6, 0xC8), false};
}

ThemePreviewCard::Palette DarkPreviewPalette() {
  return {QColor(0x1C, 0x1C, 0x1E), QColor(0x2C, 0x2C, 0x2E),
          QColor(0x28, 0x28, 0x2A), QColor(0x22, 0x22, 0x24),
          QColor(0xF2, 0xF2, 0xF7), QColor(0x48, 0x48, 0x4A),
          QColor(0x0A, 0x84, 0xFF), QColor(0x48, 0x48, 0x4A), false};
}

ThemePreviewCard::Palette LiquidPreviewPalette() {
  return {QColor(0xE8, 0xEE, 0xF6), QColor(0xEE, 0xF3, 0xFA),
          QColor(0xD8, 0xE4, 0xF4), QColor(0xF7, 0xFA, 0xFF),
          QColor(0x1A, 0x1A, 0x2E), QColor(0xB8, 0xC4, 0xD6),
          QColor(0x00, 0x7A, 0xFF), QColor(0xB8, 0xC4, 0xD6), true};
}

void SyncThemePreviewCards(QWidget* host, const QString& theme_mode) {
  if (host == nullptr) {
    return;
  }
  for (ThemePreviewCard* card : host->findChildren<ThemePreviewCard*>()) {
    if (card != nullptr) {
      card->setSelected(card->mode() == theme_mode);
    }
  }
}

}  // namespace

AppSettingsDialog::AppSettingsDialog(common::VisualizationManager* manager,
                                     QWidget* parent)
    : manager_(manager), QDialog(parent) {
  setWindowTitle(tr("Settings"));
  setModal(true);
  resize(720, 560);
  setMinimumSize(640, 480);
  setObjectName(QStringLiteral("AutovizAppSettingsDialog"));
  setStyleSheet(style::sheet(QStringLiteral("app_settings"), AppSettingsTokens()));

  const AppUiPreferences ui_prefs = LoadAppUiPreferences();

  auto* root = new QHBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  // —— Sidebar ——
  sidebar_ = new QListWidget(this);
  sidebar_->setObjectName(QStringLiteral("AutovizSettingsSidebar"));
  sidebar_->setFixedWidth(kSidebarW);
  sidebar_->setIconSize(QSize(18, 18));
  sidebar_->setSpacing(2);
  sidebar_->setUniformItemSizes(true);
  sidebar_->setSelectionMode(QAbstractItemView::SingleSelection);
  sidebar_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  sidebar_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);

  struct Category {
    const char* title;
    const char* icon_id;
  };
  const Category categories[] = {
      {QT_TR_NOOP("Appearance"), "file.image"},
      {QT_TR_NOOP("General"), "app.settings"},
      {QT_TR_NOOP("3D View"), "panels.fullscreen"},
      {QT_TR_NOOP("Layout"), "file.reset_layout"},
      {QT_TR_NOOP("Shortcuts"), "panels.tools"},
      {QT_TR_NOOP("Playback"), "file.recent"},
  };
  for (const Category& cat : categories) {
    auto* item = new QListWidgetItem(tr(cat.title));
    item->setIcon(IconLoader::menuIcon(QString::fromLatin1(cat.icon_id)));
    item->setSizeHint(QSize(kSidebarW - 16, 34));
    sidebar_->addItem(item);
  }

  // —— Detail ——
  auto* detail = new QWidget(this);
  detail->setObjectName(QStringLiteral("AutovizSettingsDetail"));
  auto* detail_layout = new QVBoxLayout(detail);
  detail_layout->setContentsMargins(0, 0, 0, 0);
  detail_layout->setSpacing(0);

  auto* header = new QWidget(detail);
  auto* header_layout = new QVBoxLayout(header);
  header_layout->setContentsMargins(28, 22, 28, 8);
  header_layout->setSpacing(0);
  page_title_ = new QLabel(header);
  page_title_->setObjectName(QStringLiteral("AutovizSettingsPageTitle"));
  header_layout->addWidget(page_title_);
  detail_layout->addWidget(header);

  pages_ = new QStackedWidget(detail);

  // Page: Language / Appearance
  {
    auto* group = MakeGroup(pages_);
    auto* gl = qobject_cast<QVBoxLayout*>(group->layout());
    language_combo_ = new QComboBox;
    language_combo_->setMinimumWidth(220);
    language_combo_->addItem(tr("Follow system"), QString());
    language_combo_->addItem(tr("English"), QStringLiteral("en"));
    language_combo_->addItem(tr("简体中文"), QStringLiteral("zh_CN"));
    gl->addWidget(MakeRow(group, tr("Interface language"), language_combo_));
    gl->addWidget(MakeHairline(group));

    theme_combo_ = new QComboBox;
    theme_combo_->setMinimumWidth(220);
    theme_combo_->addItem(tr("Follow system"), QStringLiteral("system"));
    theme_combo_->addItem(tr("Light"), QStringLiteral("light"));
    theme_combo_->addItem(tr("Dark"), QStringLiteral("dark"));
    theme_combo_->addItem(tr("Liquid Glass"), QStringLiteral("liquid"));
    gl->addWidget(MakeRow(group, tr("Theme"), theme_combo_));
    gl->addWidget(MakeHairline(group));

    theme_preview_host_ = new QWidget(group);
    theme_preview_host_->setObjectName(QStringLiteral("AutovizThemePreviewHost"));
    auto* preview_layout = new QHBoxLayout(theme_preview_host_);
    preview_layout->setContentsMargins(14, 12, 14, 14);
    preview_layout->setSpacing(12);

    struct ThemeCardSpec {
      const char* mode;
      const char* title;
      ThemePreviewCard::Palette (*palette)();
    };
    const ThemeCardSpec specs[] = {
        {"light", QT_TR_NOOP("Light"), &LightPreviewPalette},
        {"dark", QT_TR_NOOP("Dark"), &DarkPreviewPalette},
        {"liquid", QT_TR_NOOP("Liquid Glass"), &LiquidPreviewPalette},
    };
    for (const ThemeCardSpec& spec : specs) {
      auto* card = new ThemePreviewCard(QString::fromLatin1(spec.mode),
                                        tr(spec.title), spec.palette(),
                                        theme_preview_host_);
      card->onClicked = [this, mode = QString::fromLatin1(spec.mode)]() {
        if (theme_combo_ == nullptr) {
          return;
        }
        const int index = theme_combo_->findData(mode);
        if (index >= 0) {
          theme_combo_->setCurrentIndex(index);
        }
      };
      preview_layout->addWidget(card, 0, Qt::AlignTop);
    }
    preview_layout->addStretch(1);
    gl->addWidget(theme_preview_host_);
    pages_->addWidget(WrapPage(MakePageShell(pages_, group)));
  }

  // Page: General
  {
    auto* group = MakeGroup(pages_);
    auto* gl = qobject_cast<QVBoxLayout*>(group->layout());

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
    render_backend_combo_->addItem(QStringLiteral("Ogre"), QStringLiteral("Ogre"));
    render_backend_combo_->setEnabled(false);
    render_backend_combo_->setToolTip(
        tr("Autoviz uses the Ogre 1.x viewport only."));
    gl->addWidget(MakeRow(group, tr("Render backend"), render_backend_combo_));

    pages_->addWidget(WrapPage(MakePageShell(pages_, group)));
  }

  // Page: 3D View
  {
    auto* group = MakeGroup(pages_);
    auto* gl = qobject_cast<QVBoxLayout*>(group->layout());

    auto* bg_field = new QWidget;
    auto* bg_layout = new QHBoxLayout(bg_field);
    bg_layout->setContentsMargins(0, 0, 0, 0);
    bg_layout->setSpacing(8);
    background_preview_ = new QLabel(bg_field);
    background_preview_->setFixedSize(22, 22);
    background_preview_->setStyleSheet(
        style::mark(style::Mark::Preview));
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

  // Page: Layout
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

  // Page: Shortcuts
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

  // Page: Playback
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

    const QString backend = QString::fromStdString(manager_->renderBackendName());
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
  if (theme_combo_ != nullptr) {
    const int theme_index = theme_combo_->findData(ui_prefs.theme_mode);
    theme_combo_->setCurrentIndex(theme_index >= 0 ? theme_index : 1);
    syncThemePreviewSelection(theme_combo_->currentData().toString());
    const QString preview_restore = ui_prefs.theme_mode;
    // Live preview while browsing — Done persists; Cancel restores previous.
    connect(theme_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) {
              if (theme_combo_ == nullptr) {
                return;
              }
              const QString mode = theme_combo_->currentData().toString();
              syncThemePreviewSelection(mode);
              if (QApplication* app =
                      qobject_cast<QApplication*>(QApplication::instance())) {
                ApplyUiThemeMode(mode, *app);
              }
            });
    connect(this, &QDialog::rejected, this, [preview_restore]() {
      if (QApplication* app =
              qobject_cast<QApplication*>(QApplication::instance())) {
        ApplyUiThemeMode(preview_restore, *app);
      }
    });
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

void AppSettingsDialog::syncThemePreviewSelection(const QString& theme_mode) {
  // "Follow system" has no dedicated card — clears all selections.
  SyncThemePreviewCards(theme_preview_host_, theme_mode);
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
  result.theme_mode = theme_combo_ != nullptr
                          ? theme_combo_->currentData().toString()
                          : QStringLiteral("light");
  result.start_maximized = start_maximized_check_->isChecked();
  if (shortcuts_editor_ != nullptr) {
    result.shortcuts = shortcuts_editor_->shortcuts();
  }
  return result;
}

}  // namespace autoviz

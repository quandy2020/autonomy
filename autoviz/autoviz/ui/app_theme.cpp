/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/app_theme.hpp"

#include <QApplication>
#include <QColor>
#include <QFont>
#include <QPalette>
#include <QStyleFactory>

#include "autoviz/ui/autoviz_proxy_style.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace {

/**
 * Unified light shell — shared by all non-3D panels.
 * Tokens: bg #f8f9fb, surface #ffffff, accent #0891b2, text #1e293b.
 */
struct RvizPalette {
  QColor window{0xF8, 0xF9, 0xFB};
  QColor window_text{0x1E, 0x29, 0x3B};
  QColor base{0xFF, 0xFF, 0xFF};
  QColor alternate_base{0xF1, 0xF5, 0xF9};
  QColor card{0xFF, 0xFF, 0xFF};
  QColor text{0x1E, 0x29, 0x3B};
  QColor secondary_text{0x64, 0x74, 0x8B};
  QColor button{0xF1, 0xF5, 0xF9};
  QColor button_text{0x1E, 0x29, 0x3B};
  QColor accent{0x08, 0x91, 0xB2};
  QColor accent_pressed{0x0E, 0x74, 0x90};
  QColor selection_bg{0xCF, 0xFA, 0xFE};
  QColor selection_text{0x1E, 0x29, 0x3B};
  QColor link{0x08, 0x91, 0xB2};
  QColor border{0xCB, 0xD5, 0xE1};
  QColor border_light{0xE2, 0xE8, 0xF0};
  QColor tooltip_base{0xFF, 0xFF, 0xFF};
  QColor tooltip_text{0x1E, 0x29, 0x3B};
  QColor disabled_text{0x94, 0xA3, 0xB8};
  QColor toolbar_bg{0xFF, 0xFF, 0xFF};
  QColor hover_bg{0xE0, 0xF2, 0xFE};
  QColor dock_title{0xFF, 0xFF, 0xFF};
  QColor status_bar{0xF8, 0xF9, 0xFB};
};

RvizPalette RvizTheme() { return RvizPalette{}; }

QPalette BuildPaletteFromRviz(const RvizPalette& t) {
  QPalette palette;
  palette.setColor(QPalette::Window, t.window);
  palette.setColor(QPalette::WindowText, t.window_text);
  palette.setColor(QPalette::Base, t.base);
  palette.setColor(QPalette::AlternateBase, t.alternate_base);
  palette.setColor(QPalette::ToolTipBase, t.tooltip_base);
  palette.setColor(QPalette::ToolTipText, t.tooltip_text);
  palette.setColor(QPalette::Text, t.text);
  palette.setColor(QPalette::Button, t.button);
  palette.setColor(QPalette::ButtonText, t.button_text);
  palette.setColor(QPalette::BrightText, t.accent);
  palette.setColor(QPalette::Link, t.link);
  palette.setColor(QPalette::Highlight, t.selection_bg);
  palette.setColor(QPalette::HighlightedText, t.selection_text);
  palette.setColor(QPalette::Mid, t.border);
  palette.setColor(QPalette::Midlight, t.border_light);
  palette.setColor(QPalette::Disabled, QPalette::Text, t.disabled_text);
  palette.setColor(QPalette::Disabled, QPalette::ButtonText, t.disabled_text);
  palette.setColor(QPalette::Disabled, QPalette::WindowText, t.disabled_text);
  return palette;
}

QString Hex(const QColor& c) { return c.name(QColor::HexRgb); }

QString BuildPanelsMenuCheckboxIndicatorQss(const RvizPalette& t,
                                            const QString& selector_prefix) {
  return QStringLiteral(
             "%1::item {"
             "  padding: 4px 24px 4px 28px;"
             "  min-height: 20px;"
             "}"
             "%1::indicator {"
             "  width: 13px;"
             "  height: 13px;"
             "  left: 6px;"
             "  subcontrol-origin: padding;"
             "  subcontrol-position: left center;"
             "}"
             "%1::indicator:non-exclusive:unchecked,"
             "%1::indicator:unchecked {"
             "  border: 1px solid %2;"
             "  background-color: %3;"
             "  border-radius: 3px;"
             "}"
             "%1::indicator:non-exclusive:checked,"
             "%1::indicator:checked {"
             "  border: 1px solid %4;"
             "  background-color: %4;"
             "  border-radius: 3px;"
             "  image: url(:/autoviz/icons/menu_check_white.svg);"
             "}")
      .arg(selector_prefix, Hex(t.border), Hex(t.card), Hex(t.accent));
}

QString BuildRvizStylesheet(const RvizPalette& t) {
  const QString window = Hex(t.window);
  const QString base = Hex(t.base);
  const QString card = Hex(t.card);
  const QString alt = Hex(t.alternate_base);
  const QString text = Hex(t.text);
  const QString secondary = Hex(t.secondary_text);
  const QString button = Hex(t.button);
  const QString button_text = Hex(t.button_text);
  const QString accent = Hex(t.accent);
  const QString accent_pressed = Hex(t.accent_pressed);
  const QString selection_bg = Hex(t.selection_bg);
  const QString selection_text = Hex(t.selection_text);
  const QString border = Hex(t.border);
  const QString border_light = Hex(t.border_light);
  const QString disabled = Hex(t.disabled_text);
  const QString toolbar = Hex(t.toolbar_bg);
  const QString hover = Hex(t.hover_bg);
  const QString dock_title = Hex(t.dock_title);
  const QString status_bar = Hex(t.status_bar);

  const QString id_content = QLatin1String(AppThemeIds::kPanelContent);
  const QString id_toolbar = QLatin1String(AppThemeIds::kPanelToolbar);
  const QString id_footer = QLatin1String(AppThemeIds::kPanelFooter);
  const QString id_dock_title = QLatin1String(AppThemeIds::kDockTitleBar);
  const QString id_title_tools = QLatin1String(AppThemeIds::kPanelTitleTools);
  const QString id_hint = QLatin1String(AppThemeIds::kHintLabel);
  const QString id_section = QLatin1String(AppThemeIds::kSectionTitle);
  const QString id_pi_title = QLatin1String(AppThemeIds::kPropertyInspectorTitle);
  const QString id_tree = QLatin1String(AppThemeIds::kPanelTree);
  const QString id_segment = QLatin1String(AppThemeIds::kSegmentedToggle);
  const QString id_settings = QLatin1String(AppThemeIds::kSettingsScroll);

  QString css = QStringLiteral(
      /* ---- Shell ---- */
      "QMainWindow { background: __WINDOW__; }"
      "#MainPanelHost { background: __WINDOW__; }"
      "QWidget { color: __TEXT__; font-size: 13px; }"
      "QLabel { color: __TEXT__; background: transparent; }"

      "QMenuBar {"
      "  background: __TOOLBAR__;"
      "  color: __TEXT__;"
      "  border: none;"
      "  border-bottom: 1px solid __BORDER__;"
      "  padding: 0px;"
      "  spacing: 0px;"
      "}"
      "QMenuBar::item {"
      "  background: transparent;"
      "  padding: 4px 8px;"
      "}"
      "QMenuBar::item:selected { background: __HOVER__; color: __TEXT__; }"
      "QMenuBar::item:pressed { background: __HOVER__; color: __TEXT__; }"

      "QMenu {"
      "  background: __CARD__;"
      "  color: __TEXT__;"
      "  border: 1px solid __BORDER__;"
      "  padding: 2px;"
      "}"
      "QMenu::item {"
      "  padding: 4px 24px 4px 20px;"
      "  min-height: 20px;"
      "}"
      "QMenu::item:selected { background: __SEL_BG__; color: __SEL_TEXT__; }"
      "QMenu::item:disabled { color: __DISABLED__; }"
      "QMenu::separator { height: 1px; background: __BORDER__; margin: 4px 8px; }"

      "__PANELS_MENU__"

      "QToolBar {"
      "  background: __TOOLBAR__;"
      "  border: none;"
      "  border-bottom: 1px solid __BORDER__;"
      "  spacing: 2px;"
      "  padding: 2px 4px;"
      "}"
      "QToolButton {"
      "  background: transparent;"
      "  border: 1px solid transparent;"
      "  border-radius: 6px;"
      "  padding: 2px 4px;"
      "  color: __TEXT__;"
      "}"
      "QToolButton:hover { background: __HOVER__; }"
      "QToolButton:checked {"
      "  background: rgba(8,145,178,0.14); color: __ACCENT__; border-color: rgba(8,145,178,0.35);"
      "}"

      "QStatusBar {"
      "  background: __STATUS__;"
      "  color: __SECONDARY__;"
      "  border-top: 1px solid __BORDER__;"
      "  padding: 2px 8px;"
      "  font-size: 12px;"
      "}"
      "QStatusBar::item { border: none; }"

      "QTabWidget::pane {"
      "  border: none; background: __WINDOW__; top: -1px;"
      "}"
      "QTabBar::tab {"
      "  background: transparent;"
      "  color: __SECONDARY__;"
      "  border: none;"
      "  border-bottom: 2px solid transparent;"
      "  padding: 8px 14px;"
      "  margin-right: 2px;"
      "}"
      "QTabBar::tab:selected {"
      "  color: __ACCENT__; border-bottom: 2px solid __ACCENT__;"
      "}"
      "QTabBar::tab:hover:!selected { color: __TEXT__; }"

      "QDockWidget { color: __TEXT__; }"
      "QDockWidget::title {"
      "  background: __DOCK_TITLE__;"
      "  padding: 6px 8px;"
      "  border-bottom: 1px solid __BORDER__;"
      "  font-weight: 600;"
      "  text-align: left;"
      "}"

      "QGroupBox {"
      "  border: 1px solid __BORDER__;"
      "  border-radius: 10px;"
      "  margin-top: 12px;"
      "  padding: 14px 10px 10px 10px;"
      "  background: __CARD__;"
      "}"
      "QGroupBox::title {"
      "  subcontrol-origin: margin;"
      "  left: 10px;"
      "  padding: 0 6px;"
      "  color: __SECONDARY__;"
      "  font-weight: 700;"
      "  font-size: 11px;"
      "}"

      "QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QTextEdit, QPlainTextEdit {"
      "  background: __BASE__;"
      "  color: __TEXT__;"
      "  border: 1px solid __BORDER__;"
      "  border-radius: 8px;"
      "  padding: 5px 8px;"
      "  min-height: 24px;"
      "  selection-background-color: __SEL_BG__;"
      "  selection-color: __SEL_TEXT__;"
      "}"
      "QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus,"
      "QTextEdit:focus, QPlainTextEdit:focus { border: 1px solid __ACCENT__; }"
      "QComboBox::drop-down {"
      "  subcontrol-origin: padding; subcontrol-position: top right;"
      "  width: 26px; border: none;"
      "}"
      "QComboBox QAbstractItemView {"
      "  background: __BASE__;"
      "  color: __TEXT__;"
      "  border: 1px solid __BORDER__;"
      "  selection-background-color: __SEL_BG__;"
      "  selection-color: __SEL_TEXT__;"
      "}"

      "QPushButton {"
      "  background: __BUTTON__;"
      "  color: __BUTTON_TEXT__;"
      "  border: 1px solid __BORDER__;"
      "  border-radius: 8px;"
      "  padding: 5px 14px;"
      "  min-height: 22px;"
      "  font-weight: 600;"
      "}"
      "QPushButton:hover { background: __HOVER__; border-color: __ACCENT__; color: __ACCENT__; }"
      "QPushButton:pressed { background: rgba(8,145,178,0.22); color: __ACCENT__; }"
      "QPushButton:disabled { color: __DISABLED__; border-color: __BORDER_LIGHT__; }"
      "QPushButton:default {"
      "  background: __ACCENT__; color: white; border-color: __ACCENT__; font-weight: 700;"
      "}"
      "QPushButton:default:hover { background: __ACCENT_PRESSED__; }"
      "QPushButton:flat { background: transparent; border: none; color: __ACCENT__; }"
      "QPushButton:flat:hover { background: __HOVER__; }"

      "QCheckBox, QRadioButton { spacing: 6px; color: __TEXT__; }"

      "QSlider::groove:horizontal {"
      "  height: 6px; background: __BORDER__; border-radius: 3px;"
      "}"
      "QSlider::handle:horizontal {"
      "  width: 14px; height: 14px; margin: -4px 0;"
      "  background: __ACCENT__; border-radius: 7px;"
      "}"
      "QProgressBar {"
      "  background: __BORDER_LIGHT__; border: 1px solid __BORDER__;"
      "  border-radius: 6px; text-align: center; color: __TEXT__;"
      "}"
      "QProgressBar::chunk { background: __ACCENT__; border-radius: 5px; }"

      "QTreeWidget, QTreeView, QListWidget, QListView, QTableWidget, QTableView {"
      "  background: __BASE__;"
      "  alternate-background-color: __ALT__;"
      "  color: __TEXT__;"
      "  border: 1px solid __BORDER__;"
      "  border-radius: 0px;"
      "  outline: none;"
      "}"
      "QTreeWidget::item, QTreeView::item, QListWidget::item, QListView::item,"
      "QTableWidget::item, QTableView::item {"
      "  padding: 4px 6px; min-height: 22px;"
      "}"
      "QTreeWidget::item:hover, QTreeView::item:hover, QListWidget::item:hover,"
      "QListView::item:hover { background: rgba(15,23,42,0.04); }"
      "QTreeWidget::item:selected, QTreeView::item:selected, QListWidget::item:selected,"
      "QListView::item:selected, QTableWidget::item:selected, QTableView::item:selected {"
      "  background: rgba(8,145,178,0.14); color: __TEXT__;"
      "}"
      "QHeaderView::section {"
      "  background: __WINDOW__;"
      "  color: __SECONDARY__;"
      "  border: none;"
      "  border-bottom: 1px solid __BORDER__;"
      "  padding: 7px 8px;"
      "  font-size: 11px;"
      "  font-weight: 700;"
      "}"

      "QScrollBar:vertical { background: __WINDOW__; width: 10px; margin: 0; }"
      "QScrollBar::handle:vertical {"
      "  background: #94a3b8; min-height: 24px; border-radius: 5px; margin: 2px;"
      "}"
      "QScrollBar::handle:vertical:hover { background: __ACCENT__; }"
      "QScrollBar:horizontal { background: __WINDOW__; height: 10px; margin: 0; }"
      "QScrollBar::handle:horizontal {"
      "  background: #94a3b8; min-width: 24px; border-radius: 5px; margin: 2px;"
      "}"
      "QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }"

      "QSplitter::handle { background: __BORDER__; }"
      "QSplitter::handle:horizontal { width: 1px; }"
      "QSplitter::handle:vertical { height: 1px; }"

      "QDialog, QDialogButtonBox { background: __WINDOW__; }"
      "QScrollArea { background: __WINDOW__; border: none; }"
      "QScrollArea > QWidget > QWidget { background: __WINDOW__; }"

      /* ---- Named panel chrome ---- */
      "#__ID_CONTENT__ { background: __WINDOW__; color: __TEXT__; }"
      "#__ID_TOOLBAR__ {"
      "  background: __TOOLBAR__; border-bottom: 1px solid __BORDER__;"
      "}"
      "#__ID_FOOTER__ {"
      "  background: __WINDOW__; border-top: 1px solid __BORDER__;"
      "}"
      "#__ID_DOCK_TITLE__ {"
      "  background: __DOCK_TITLE__; border-bottom: 1px solid __BORDER__;"
      "}"
      "#__ID_DOCK_TITLE__ QLabel {"
      "  font-weight: 600; color: __TEXT__; padding: 0 4px;"
      "}"
      "#__ID_TITLE_TOOLS__ QToolButton {"
      "  border: 1px solid transparent; padding: 2px; background: transparent;"
      "  border-radius: 6px;"
      "}"
      "#__ID_TITLE_TOOLS__ QToolButton:hover {"
      "  background: __HOVER__; border-color: __BORDER_LIGHT__;"
      "}"
      "#__ID_TITLE_TOOLS__ QToolButton:checked {"
      "  background: rgba(8,145,178,0.14); color: __ACCENT__;"
      "  border-color: rgba(8,145,178,0.35);"
      "}"
      "#__ID_HINT__ {"
      "  color: __SECONDARY__; font-size: 11px;"
      "}"
      "#__ID_SECTION__ {"
      "  font-size: 13px; font-weight: 700; color: __TEXT__; background: transparent;"
      "}"
      "#__ID_PI_TITLE__ {"
      "  font-weight: 700; padding: 6px 8px; color: __TEXT__;"
      "  border-bottom: 1px solid __BORDER__; background: __WINDOW__;"
      "}"
      "QTreeWidget#__ID_TREE__, QTreeView#__ID_TREE__ {"
      "  background: __BASE__; color: __TEXT__; border: none;"
      "  alternate-background-color: __ALT__;"
      "}"
      "QTreeWidget#__ID_TREE__::item:selected, QTreeView#__ID_TREE__::item:selected {"
      "  background: rgba(8,145,178,0.14); color: __TEXT__;"
      "}"
      "#__ID_SEGMENT__ {"
      "  background: __ALT__; border: 1px solid __BORDER__; border-radius: 8px; padding: 2px;"
      "}"
      "#__ID_SEGMENT__ QPushButton {"
      "  border: none; padding: 4px 10px; background: transparent; border-radius: 6px;"
      "}"
      "#__ID_SEGMENT__ QPushButton:checked {"
      "  background: __BASE__; color: __ACCENT__; font-weight: 700;"
      "}"
      "#__ID_SETTINGS__ { border: none; background: __WINDOW__; }"

      "QToolTip {"
      "  background: __CARD__; color: __TEXT__; border: 1px solid __BORDER__;"
      "  padding: 4px 8px; border-radius: 6px;"
      "}");

  const auto replace_all = [&css](const char* key, const QString& value) {
    css.replace(QLatin1String(key), value);
  };
  replace_all("__WINDOW__", window);
  replace_all("__BASE__", base);
  replace_all("__CARD__", card);
  replace_all("__ALT__", alt);
  replace_all("__TEXT__", text);
  replace_all("__SECONDARY__", secondary);
  replace_all("__BUTTON__", button);
  replace_all("__BUTTON_TEXT__", button_text);
  replace_all("__ACCENT__", accent);
  replace_all("__ACCENT_PRESSED__", accent_pressed);
  replace_all("__SEL_BG__", selection_bg);
  replace_all("__SEL_TEXT__", selection_text);
  replace_all("__BORDER__", border);
  replace_all("__BORDER_LIGHT__", border_light);
  replace_all("__DISABLED__", disabled);
  replace_all("__TOOLBAR__", toolbar);
  replace_all("__HOVER__", hover);
  replace_all("__DOCK_TITLE__", dock_title);
  replace_all("__STATUS__", status_bar);
  replace_all("__ID_CONTENT__", id_content);
  replace_all("__ID_TOOLBAR__", id_toolbar);
  replace_all("__ID_FOOTER__", id_footer);
  replace_all("__ID_DOCK_TITLE__", id_dock_title);
  replace_all("__ID_TITLE_TOOLS__", id_title_tools);
  replace_all("__ID_HINT__", id_hint);
  replace_all("__ID_SECTION__", id_section);
  replace_all("__ID_PI_TITLE__", id_pi_title);
  replace_all("__ID_TREE__", id_tree);
  replace_all("__ID_SEGMENT__", id_segment);
  replace_all("__ID_SETTINGS__", id_settings);
  replace_all("__PANELS_MENU__",
              BuildPanelsMenuCheckboxIndicatorQss(
                  t, QStringLiteral("QMenu#") +
                         QLatin1String(AppThemeIds::kPanelsMenu)));
  return css;
}

void ApplyRvizFont(QApplication& app) {
  QFont font = app.font();
  font.setFamilies({QStringLiteral("Segoe UI"), QStringLiteral("Ubuntu"),
                    QStringLiteral("Cantarell"), QStringLiteral("Noto Sans"),
                    QStringLiteral("Sans Serif")});
  font.setPointSizeF(9.0);
  app.setFont(font);
}

}  // namespace

QColor AppThemeAccentColor() { return RvizTheme().accent; }

QColor AppThemeSuggestedViewportBackground() { return QColor(48, 48, 48); }

QPalette BuildThemePalette() { return BuildPaletteFromRviz(RvizTheme()); }

QString BuildAppThemeStylesheet() { return BuildRvizStylesheet(RvizTheme()); }

QString BuildPanelsMenuStylesheet() {
  return BuildPanelsMenuCheckboxIndicatorQss(RvizTheme(), QStringLiteral("QMenu"));
}

void ApplyAppTheme(QApplication& app) {
  ApplyRvizFont(app);
  QStyle* fusion = QStyleFactory::create(QStringLiteral("Fusion"));
  auto* proxy = new AutovizProxyStyle(fusion);
  proxy->setParent(&app);

  const RvizPalette theme = RvizTheme();
  proxy->setAccentColor(theme.accent);
  proxy->setSelectionBackground(theme.selection_bg);
  proxy->setHoverBackground(theme.hover_bg);
  app.setStyle(proxy);
  app.setPalette(BuildPaletteFromRviz(theme));
  app.setStyleSheet(BuildRvizStylesheet(theme));
}

}  // namespace autoviz

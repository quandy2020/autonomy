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

/** Qt Fusion light palette — matches RViz2 default shell (no custom dark QSS). */
struct RvizPalette {
  QColor window{0xF0, 0xF0, 0xF0};
  QColor window_text{0x00, 0x00, 0x00};
  QColor base{0xFF, 0xFF, 0xFF};
  QColor alternate_base{0xE9, 0xE9, 0xE9};
  QColor card{0xFF, 0xFF, 0xFF};
  QColor text{0x00, 0x00, 0x00};
  QColor secondary_text{0x64, 0x64, 0x64};
  QColor button{0xF0, 0xF0, 0xF0};
  QColor button_text{0x00, 0x00, 0x00};
  QColor accent{0x30, 0x8C, 0xC6};
  QColor accent_pressed{0x26, 0x8A, 0xB2};
  QColor selection_bg{0x30, 0x8C, 0xC6};
  QColor selection_text{0xFF, 0xFF, 0xFF};
  QColor link{0x00, 0x00, 0xFF};
  QColor border{0xC0, 0xC0, 0xC0};
  QColor border_light{0xD4, 0xD4, 0xD4};
  QColor tooltip_base{0xFF, 0xFF, 0xDC};
  QColor tooltip_text{0x00, 0x00, 0x00};
  QColor disabled_text{0x80, 0x80, 0x80};
  QColor toolbar_bg{0xF0, 0xF0, 0xF0};
  QColor hover_bg{0xE5, 0xF3, 0xFF};
  QColor dock_title{0xA0, 0xA0, 0xA0};
  QColor status_bar{0xF0, 0xF0, 0xF0};
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

QString BuildPanelsMenuCheckboxIndicatorQss(const RvizPalette& t,
                                            const QString& selector_prefix) {
  const auto hex = [](const QColor& c) { return c.name(QColor::HexRgb); };
  const QString card = hex(t.card);
  const QString accent = hex(t.accent);

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
             "  border: 1px solid #767676;"
             "  background-color: %2;"
             "  border-radius: 2px;"
             "}"
             "%1::indicator:non-exclusive:checked,"
             "%1::indicator:checked {"
             "  border: 1px solid %3;"
             "  background-color: %3;"
             "  border-radius: 2px;"
             "  image: url(:/autoviz/icons/menu_check_white.svg);"
             "}")
      .arg(selector_prefix, card, accent);
}

QString BuildRvizStylesheet(const RvizPalette& t) {
  const auto hex = [](const QColor& c) { return c.name(QColor::HexRgb); };

  const QString window = hex(t.window);
  const QString base = hex(t.base);
  const QString card = hex(t.card);
  const QString alt = hex(t.alternate_base);
  const QString text = hex(t.text);
  const QString secondary = hex(t.secondary_text);
  const QString button = hex(t.button);
  const QString button_text = hex(t.button_text);
  const QString accent = hex(t.accent);
  const QString accent_pressed = hex(t.accent_pressed);
  const QString selection_bg = hex(t.selection_bg);
  const QString selection_text = hex(t.selection_text);
  const QString border = hex(t.border);
  const QString border_light = hex(t.border_light);
  const QString disabled = hex(t.disabled_text);
  const QString toolbar = hex(t.toolbar_bg);
  const QString hover = hex(t.hover_bg);
  const QString dock_title = hex(t.dock_title);
  const QString status_bar = hex(t.status_bar);

  return QStringLiteral(
             /* ---- Shell (RViz2 / Fusion light) ---- */
             "QMainWindow { background: %1; }"
             "#MainPanelHost { background: %1; }"
             "QWidget { color: %3; font-size: 13px; }"
             "QLabel { color: %3; background: transparent; }"

             "QMenuBar {"
             "  background: %12;"
             "  color: %3;"
             "  border: none;"
             "  border-bottom: 1px solid %10;"
             "  padding: 0px;"
             "  spacing: 0px;"
             "}"
             "QMenuBar::item {"
             "  background: transparent;"
             "  padding: 4px 8px;"
             "}"
             "QMenuBar::item:selected { background: %17; color: %3; }"
             "QMenuBar::item:pressed { background: %17; color: %3; }"

             "QMenu {"
             "  background: %5;"
             "  color: %3;"
             "  border: 1px solid %10;"
             "  padding: 2px;"
             "}"
             "QMenu::item {"
             "  padding: 4px 24px 4px 20px;"
             "  min-height: 20px;"
             "}"
             "QMenu::item:selected { background: %8; color: %9; }"
             "QMenu::item:disabled { color: %14; }"
             "QMenu::separator { height: 1px; background: %10; margin: 4px 8px; }"

             "%33"

             "QToolBar {"
             "  background: %12;"
             "  border: none;"
             "  border-bottom: 1px solid %10;"
             "  spacing: 2px;"
             "  padding: 2px 4px;"
             "}"
             "QToolButton {"
             "  background: transparent;"
             "  border: 1px solid transparent;"
             "  border-radius: 2px;"
             "  padding: 1px 3px;"
             "  color: %3;"
             "}"
             "QToolButton:hover { background: %17; }"
             "QToolButton:checked { background: %6; color: %3; border-color: %10; }"

             "QStatusBar {"
             "  background: %18;"
             "  color: %4;"
             "  border-top: 1px solid %10;"
             "  padding: 2px 8px;"
             "  font-size: 12px;"
             "}"
             "QStatusBar::item { border: none; }"

             "QTabWidget::pane { border: 1px solid %10; background: %1; top: -1px; }"
             "QTabBar::tab {"
             "  background: %6;"
             "  color: %3;"
             "  border: 1px solid %10;"
             "  border-bottom: none;"
             "  padding: 6px 14px;"
             "  margin-right: 2px;"
             "  border-top-left-radius: 3px;"
             "  border-top-right-radius: 3px;"
             "}"
             "QTabBar::tab:selected { background: %5; border-bottom: 1px solid %5; }"
             "QTabBar::tab:hover:!selected { background: %17; }"

             "QDockWidget { color: %3; }"
             "QDockWidget::title {"
             "  background: %19;"
             "  padding: 4px 6px;"
             "  border-bottom: 1px solid %10;"
             "  font-weight: 600;"
             "  text-align: left;"
             "}"

             "QGroupBox {"
             "  border: 1px solid %10;"
             "  border-radius: 4px;"
             "  margin-top: 12px;"
             "  padding: 14px 8px 8px 8px;"
             "  background: %5;"
             "}"
             "QGroupBox::title {"
             "  subcontrol-origin: margin;"
             "  left: 8px;"
             "  padding: 0 4px;"
             "  color: %3;"
             "}"

             "QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QTextEdit, QPlainTextEdit {"
             "  background: %2;"
             "  color: %3;"
             "  border: 1px solid %10;"
             "  border-radius: 3px;"
             "  padding: 4px 6px;"
             "  min-height: 22px;"
             "  selection-background-color: %8;"
             "  selection-color: %9;"
             "}"
             "QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus,"
             "QTextEdit:focus, QPlainTextEdit:focus { border: 1px solid %11; }"
             "QComboBox QAbstractItemView {"
             "  background: %2;"
             "  color: %3;"
             "  border: 1px solid %10;"
             "  selection-background-color: %8;"
             "  selection-color: %9;"
             "}"

             "QPushButton {"
             "  background: %6;"
             "  color: %7;"
             "  border: 1px solid %10;"
             "  border-radius: 4px;"
             "  padding: 5px 14px;"
             "  min-height: 22px;"
             "}"
             "QPushButton:hover { background: %17; border-color: %11; }"
             "QPushButton:pressed { background: %8; color: %9; }"
             "QPushButton:disabled { color: %14; border-color: %10; }"
             "QPushButton:default { background: %11; color: %9; border-color: %11; font-weight: 600; }"
             "QPushButton:default:hover { background: %15; }"
             "QPushButton:flat { background: transparent; border: none; color: %11; }"
             "QPushButton:flat:hover { background: %17; }"

             "QCheckBox, QRadioButton { spacing: 6px; color: %3; }"

             "QSlider::groove:horizontal { height: 6px; background: %10; border-radius: 3px; }"
             "QSlider::handle:horizontal {"
             "  width: 14px; height: 14px; margin: -4px 0;"
             "  background: %11; border-radius: 7px;"
             "}"
             "QProgressBar {"
             "  background: %10; border: 1px solid %10; border-radius: 3px; text-align: center;"
             "}"
             "QProgressBar::chunk { background: %11; border-radius: 2px; }"

             "QTreeWidget, QTreeView, QListWidget, QListView, QTableWidget, QTableView {"
             "  background: %2;"
             "  alternate-background-color: %16;"
             "  color: %3;"
             "  border: 1px solid %10;"
             "  outline: none;"
             "}"
             "QTreeWidget::item, QTreeView::item, QListWidget::item, QListView::item,"
             "QTableWidget::item, QTableView::item { padding: 3px 4px; min-height: 18px; }"
             "QTreeWidget::item:hover, QTreeView::item:hover, QListWidget::item:hover,"
             "QListView::item:hover { background: %17; }"
             "QTreeWidget::item:selected, QTreeView::item:selected, QListWidget::item:selected,"
             "QListView::item:selected, QTableWidget::item:selected, QTableView::item:selected {"
             "  background: %8; color: %9;"
             "}"
             "QHeaderView::section {"
             "  background: %12;"
             "  color: %3;"
             "  border: none;"
             "  border-right: 1px solid %10;"
             "  border-bottom: 1px solid %10;"
             "  padding: 4px 6px;"
             "  font-weight: 600;"
             "}"

             "QScrollBar:vertical { background: %1; width: 12px; margin: 0; }"
             "QScrollBar::handle:vertical {"
             "  background: %10; min-height: 24px; border-radius: 4px; margin: 2px;"
             "}"
             "QScrollBar::handle:vertical:hover { background: %11; }"
             "QScrollBar:horizontal { background: %1; height: 12px; margin: 0; }"
             "QScrollBar::handle:horizontal {"
             "  background: %10; min-width: 24px; border-radius: 4px; margin: 2px;"
             "}"
             "QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; }"

             "QSplitter::handle { background: %10; }"
             "QSplitter::handle:horizontal { width: 2px; }"
             "QSplitter::handle:vertical { height: 2px; }"

             "QDialog, QDialogButtonBox { background: %1; }"
             "QScrollArea { background: %1; border: none; }"
             "QScrollArea > QWidget > QWidget { background: %1; }"

             "#%31 { background: %1; }"
             "#%20, #%21 { background: %1; border-bottom: 1px solid %10; }"
             "#%22 { background: %1; border-top: 1px solid %10; }"
             "#%23 { background: %19; border-bottom: 1px solid %10; }"
             "#%23 QLabel { font-weight: 600; color: %3; padding: 0 4px; }"
             "#%24 QToolButton { border: 1px solid transparent; padding: 2px; background: transparent; }"
             "#%24 QToolButton:hover { background: %17; border-color: %13; }"
             "#%24 QToolButton:checked { background: %8; color: %9; border-color: %11; }"
             "#%25 { color: %4; font-size: 11px; font-style: italic; }"
             "#%26 { font-size: 14px; font-weight: 600; color: %3; background: transparent; }"
             "#%27 {"
             "  font-weight: 600; padding: 4px 6px; color: %3;"
             "  border-bottom: 1px solid %10; background: %1;"
             "}"
             "QTreeWidget#%28, QTreeView#%28 {"
             "  background: %2; color: %3; border: none; alternate-background-color: %16;"
             "}"
             "QTreeWidget#%28::item:selected, QTreeView#%28::item:selected {"
             "  background: %8; color: %9;"
             "}"
             "#%29 { background: %16; border: 1px solid %10; border-radius: 4px; padding: 2px; }"
             "#%29 QPushButton { border: none; padding: 4px 10px; background: transparent; }"
             "#%29 QPushButton:checked { background: %2; font-weight: 600; }"
             "#%30 { border: none; background: %1; }"

             "QToolTip {"
             "  background: %5; color: %3; border: 1px solid %10; padding: 4px 8px;"
             "}")
      .arg(window, base, text, secondary, card, button, button_text, selection_bg,
           selection_text, border, accent, toolbar, border_light, disabled, accent_pressed,
           alt, hover, status_bar, dock_title, AppThemeIds::kPanelToolbar,
           AppThemeIds::kPanelFooter, AppThemeIds::kDockTitleBar,
           AppThemeIds::kPanelTitleTools, AppThemeIds::kHintLabel,
           AppThemeIds::kSectionTitle, AppThemeIds::kPropertyInspectorTitle,
           AppThemeIds::kPanelTree, AppThemeIds::kSegmentedToggle,
           AppThemeIds::kSettingsScroll, AppThemeIds::kPanelContent,
           BuildPanelsMenuCheckboxIndicatorQss(
               t, QStringLiteral("QMenu#") + QLatin1String(AppThemeIds::kPanelsMenu)));
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

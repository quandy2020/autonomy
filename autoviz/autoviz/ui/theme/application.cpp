/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/theme/application.hpp"

#include <QApplication>
#include <QColor>
#include <QFont>
#include <QFontMetrics>
#include <QGuiApplication>
#include <QKeySequence>
#include <QMenu>
#include <QPalette>
#include <QPointer>
#include <QStyle>
#include <QStyleFactory>
#include <QStyleHints>
#include <QtGlobal>

#include "autoviz/ui/theme/proxy.hpp"
#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"

#include <QHash>

namespace autoviz {
namespace {

/**
 * Frosted-glass shell — cool light surfaces + pale cyan accents.
 * Overlay (3D HUD / floating tools) shares the same cyan language via glass::.
 */

QPalette BuildPaletteFromShell(const glass::ShellTokens& t) {
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

QString Hex(const QColor& c) { return glass::Hex(c); }

QString BuildPanelsMenuCheckboxIndicatorQss(const glass::ShellTokens& t,
                                            const QString& selector_prefix,
                                            bool dark) {
  // Spec: milky glass panels; teal hover & accent (match Displays / Add Panel).
  const QColor bg = dark ? QColor(0x24, 0x28, 0x30) : QColor(0xFF, 0xFF, 0xFF);
  const QColor text = dark ? QColor(0xE2, 0xE8, 0xF0) : QColor(0x1E, 0x29, 0x3B);
  const QColor border = dark ? QColor(0x3A, 0x40, 0x4A) : QColor(0xCB, 0xD5, 0xE1);
  const QColor hover = dark ? QColor(0x37, 0x41, 0x51) : QColor(20, 184, 166, 48);
  const QColor separator =
      dark ? QColor(0x3A, 0x40, 0x4A) : QColor(0xE2, 0xE8, 0xF0);
  Q_UNUSED(hover);
  Q_UNUSED(t);
  QHash<QString, QString> tokens;
  tokens.insert(QStringLiteral("{{root}}"), selector_prefix);
  tokens.insert(QStringLiteral("{{bg}}"), Hex(bg));
  tokens.insert(QStringLiteral("{{text}}"), Hex(text));
  tokens.insert(QStringLiteral("{{border}}"), Hex(border));
  tokens.insert(QStringLiteral("{{sep}}"), Hex(separator));
  return style::sheet(QStringLiteral("menu"), tokens);
}

bool PreferDarkPanelsMenu() {
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
  if (QGuiApplication::styleHints() != nullptr) {
    return QGuiApplication::styleHints()->colorScheme() == Qt::ColorScheme::Dark;
  }
#endif
  const QColor window = QApplication::palette().color(QPalette::Window);
  return window.lightness() < 128;
}

QString BuildShellStylesheet(const glass::ShellTokens& /*t*/) {
  return style::sheet(QStringLiteral("theme"));
}

void ApplyAppFont(QApplication& app) {
  QFont font = app.font();
  font.setFamilies({QStringLiteral("Segoe UI"), QStringLiteral("Ubuntu"),
                    QStringLiteral("Cantarell"), QStringLiteral("Noto Sans"),
                    QStringLiteral("Sans Serif")});
  font.setPointSizeF(9.0);
  app.setFont(font);
}

}  // namespace

QColor AppThemeAccentColor() { return glass::Shell().accent; }

QColor AppThemeSuggestedViewportBackground() { return QColor(48, 48, 48); }

QPalette BuildThemePalette() { return BuildPaletteFromShell(glass::Shell()); }

QString BuildAppThemeStylesheet() { return BuildShellStylesheet(glass::Shell()); }

QString BuildPanelsMenuStylesheet() {
  return BuildPanelsMenuCheckboxIndicatorQss(
      glass::Shell(), QStringLiteral("QMenu"), PreferDarkPanelsMenu());
}

QColor PanelsMenuIconInk() {
  // Match glass Shell secondary text — soft slate, not harsh black.
  return PreferDarkPanelsMenu() ? QColor(0xE2, 0xE8, 0xF0)
                                : QColor(0x47, 0x55, 0x69);
}

namespace {
QPointer<QMenu> g_active_glass_menu;
}  // namespace

bool IsActiveGlassMenuPopup() {
  return g_active_glass_menu != nullptr && g_active_glass_menu->isVisible();
}

bool IsActivePanelsMenuPopup() { return IsActiveGlassMenuPopup(); }

void PrepareAppMenu(QMenu* menu) {
  if (menu == nullptr) {
    return;
  }
  if (menu->objectName().isEmpty()) {
    menu->setObjectName(QString::fromLatin1(AppThemeIds::kAppMenu));
  }
  menu->setProperty("autovizGlassMenu", true);
  // Clear stylesheet so ProxyStyle owns CE_MenuItem / PE_PanelMenu.
  menu->setStyleSheet(QString());
  // Opaque rectangular popup chrome hides the glass round-rect corners.
  menu->setAttribute(Qt::WA_TranslucentBackground, true);
  menu->setAttribute(Qt::WA_NoSystemBackground, true);
  menu->setAutoFillBackground(false);
  menu->setWindowFlag(Qt::FramelessWindowHint, true);
  menu->setWindowFlag(Qt::NoDropShadowWindowHint, true);
  {
    QPalette pal = menu->palette();
    pal.setBrush(QPalette::Base, Qt::transparent);
    pal.setBrush(QPalette::Window, Qt::transparent);
    menu->setPalette(pal);
  }
  QFont font = menu->font();
  font.setFamilies({QStringLiteral("Inter"), QStringLiteral("Segoe UI"),
                    QStringLiteral("Roboto"), QStringLiteral("Noto Sans"),
                    QStringLiteral("Sans Serif")});
  font.setPixelSize(14);
  menu->setFont(font);
  menu->setToolTipsVisible(true);

  QObject::connect(menu, &QMenu::aboutToShow, menu, [menu]() {
    g_active_glass_menu = menu;
    const QFontMetrics fm(menu->font());
    QFont bold_font = menu->font();
    bold_font.setWeight(QFont::DemiBold);
    const QFontMetrics fmb(bold_font);
    auto measure = [](const QFontMetrics& metrics, const QString& text) {
      return qMax(metrics.horizontalAdvance(text),
                  metrics.boundingRect(text).width());
    };
    const bool panels = menu->property("autovizPanelsMenu").toBool();
    // Must match ProxyStyle layout constants.
    const int leading =
        panels ? (12 + 16 + 8 + 14 + 10)  // pad+icon+gap+check+gap
               : (12 + 16 + 10);         // pad+icon+gap
    constexpr int kPadR = 28;
    constexpr int kSubmenuW = 12;
    constexpr int kShortcutGap = 36;
    constexpr int kShortcutSafety = 18;
    int max_label = 0;
    int max_shortcut = 0;
    bool need_submenu = false;
    for (QAction* action : menu->actions()) {
      if (action == nullptr || action->isSeparator()) {
        continue;
      }
      QString label = action->text();
      QString shortcut;
      const int tab = label.indexOf(QLatin1Char('\t'));
      if (tab >= 0) {
        shortcut = label.mid(tab + 1);
        label = label.left(tab);
      }
      label.remove(QLatin1Char('&'));
      max_label = qMax(max_label, measure(fm, label));
      max_label = qMax(max_label, measure(fmb, label));
      if (!action->shortcut().isEmpty()) {
        const QString native =
            action->shortcut().toString(QKeySequence::NativeText);
        const QString portable =
            action->shortcut().toString(QKeySequence::PortableText);
        max_shortcut =
            qMax(max_shortcut, measure(fm, native) + kShortcutSafety);
        max_shortcut =
            qMax(max_shortcut, measure(fm, portable) + kShortcutSafety);
      }
      if (!shortcut.isEmpty()) {
        max_shortcut = qMax(max_shortcut, measure(fm, shortcut) + kShortcutSafety);
      }
      if (action->menu() != nullptr) {
        need_submenu = true;
      }
    }
    menu->setProperty("panelsMaxLabelW", max_label);
    menu->setProperty("glassMaxShortcutW", max_shortcut);
    const int hmargin =
        2 * menu->style()->pixelMetric(QStyle::PM_MenuHMargin, nullptr, menu);
    const int shortcut_w =
        max_shortcut > 0 ? (kShortcutGap + max_shortcut) : 0;
    // Prefer minimumWidth so style sizeFromContents can still grow the popup.
    const int width = leading + max_label + shortcut_w +
                      (need_submenu ? kSubmenuW : 0) + kPadR + hmargin + 40;
    // Clear any previous setFixedWidth so shortcuts are not clipped.
    menu->setMinimumWidth(width);
    menu->setMaximumWidth(QWIDGETSIZE_MAX);
  });
  QObject::connect(menu, &QMenu::aboutToHide, menu, [menu]() {
    if (g_active_glass_menu == menu) {
      g_active_glass_menu.clear();
    }
  });
}

void PreparePanelsMenu(QMenu* menu) {
  if (menu == nullptr) {
    return;
  }
  menu->setObjectName(QString::fromLatin1(AppThemeIds::kPanelsMenu));
  menu->setProperty("autovizPanelsMenu", true);
  PrepareAppMenu(menu);
}

void ApplyAppTheme(QApplication& app) {
  ApplyAppFont(app);
  QStyle* fusion = QStyleFactory::create(QStringLiteral("Fusion"));
  auto* proxy = new ProxyStyle(fusion);
  proxy->setParent(&app);

  const glass::ShellTokens theme = glass::Shell();
  const bool dark_menu = PreferDarkPanelsMenu();
  proxy->setAccentColor(dark_menu ? theme.accent : QColor(0x14, 0xB8, 0xA6));
  proxy->setSelectionBackground(dark_menu ? QColor(0x37, 0x41, 0x51)
                                          : QColor(20, 184, 166, 78));
  proxy->setHoverBackground(dark_menu ? QColor(0x37, 0x41, 0x51)
                                      : QColor(165, 243, 252, 56));
  proxy->setMenuShortcutColor(dark_menu ? QColor(0x94, 0xA3, 0xB8)
                                        : theme.secondary_text);
  app.setStyle(proxy);
  app.setPalette(BuildPaletteFromShell(theme));
  app.setStyleSheet(BuildShellStylesheet(theme));
}

}  // namespace autoviz

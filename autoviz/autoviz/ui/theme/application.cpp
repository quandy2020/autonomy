/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/theme/application.hpp"

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QPushButton>
#include <QApplication>
#include <QCheckBox>
#include <QFrame>
#include <QToolBar>
#include <QMenuBar>
#include <QStatusBar>
#include <QColor>
#include <QFont>
#include <QFontMetrics>
#include <QGuiApplication>
#include <QHash>
#include <QKeySequence>
#include <QMenu>
#include <QPalette>
#include <QPointer>
#include <QRadioButton>
#include <QStyle>
#include <QStyleFactory>
#include <QStyleHints>
#include <QWidget>
#include <QtGlobal>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"
#include "autoviz/ui/theme/tokens.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/proxy.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

/**
 * Apple-style shell — neutral grouped surfaces with a dynamic blue tint.
 * Overlay (3D HUD / floating tools) retains its viewport-specific palette.
 */

QPalette BuildPaletteFromShell(const theme::ShellTokens& t) {
  QPalette palette;
  // Opaque roles only — translucent Window/Base make Fusion indicators vanish.
  QColor window = t.window;
  window.setAlpha(255);
  QColor base = t.base;
  base.setAlpha(255);
  QColor button = t.button;
  button.setAlpha(255);
  QColor border = t.border;
  border.setAlpha(255);
  palette.setColor(QPalette::Window, window);
  palette.setColor(QPalette::WindowText, t.window_text);
  palette.setColor(QPalette::Base, base);
  palette.setColor(QPalette::AlternateBase, t.alternate_base);
  QColor tip_base = t.tooltip_base;
  tip_base.setAlpha(255);
  QColor tip_text = t.tooltip_text;
  tip_text.setAlpha(255);
  palette.setColor(QPalette::ToolTipBase, tip_base);
  palette.setColor(QPalette::ToolTipText, tip_text);
  palette.setColor(QPalette::Text, t.text);
  palette.setColor(QPalette::Button, button);
  palette.setColor(QPalette::ButtonText, t.button_text);
  palette.setColor(QPalette::BrightText, t.accent);
  palette.setColor(QPalette::Link, t.link);
  // Soft selection for text; checkbox chrome paints Shell.accent itself.
  palette.setColor(QPalette::Highlight, t.selection_bg);
  palette.setColor(QPalette::HighlightedText, t.selection_text);
  palette.setColor(QPalette::Mid, border);
  palette.setColor(QPalette::Midlight, t.border_light);
  palette.setColor(QPalette::Light, base);
  palette.setColor(QPalette::Dark, border);
  palette.setColor(QPalette::Disabled, QPalette::Text, t.disabled_text);
  palette.setColor(QPalette::Disabled, QPalette::ButtonText, t.disabled_text);
  palette.setColor(QPalette::Disabled, QPalette::WindowText, t.disabled_text);
  return palette;
}

QString Hex(const QColor& c) { return theme::Hex(c); }

QString BuildPanelsMenuCheckboxIndicatorQss(const theme::ShellTokens& t,
                                            const QString& selector_prefix,
                                            bool dark) {
  Q_UNUSED(dark);
  // Follow Shell (Liquid / Light / Dark) — no legacy teal/slate hardcodes.
  QHash<QString, QString> tokens;
  tokens.insert(QStringLiteral("{{root}}"), selector_prefix);
  tokens.insert(QStringLiteral("{{bg}}"), Hex(t.card));
  tokens.insert(QStringLiteral("{{text}}"), Hex(t.text));
  tokens.insert(QStringLiteral("{{border}}"), Hex(t.border));
  tokens.insert(QStringLiteral("{{sep}}"), Hex(t.border_light));
  return style::sheet(QStringLiteral("menu"), tokens);
}

bool PreferDarkPanelsMenu() { return theme::ShellIsDark(); }

QString BuildShellStylesheet(const theme::ShellTokens& /*t*/) {
  return style::sheet(QStringLiteral("theme"));
}

void ApplyAppFont(QApplication& app) {
  QFont font = app.font();
  font.setFamilies({QStringLiteral(".AppleSystemUIFont"),
                    QStringLiteral("SF Pro Text"),
                    QStringLiteral("Helvetica Neue"), QStringLiteral("Inter"),
                    QStringLiteral("Liberation Sans"),
                    QStringLiteral("Noto Sans"), QStringLiteral("Sans Serif")});
  font.setPointSizeF(10.5);
  app.setFont(font);
}

}  // namespace

QColor AppThemeAccentColor() { return theme::Shell().accent; }

QColor AppThemeSuggestedViewportBackground() { return QColor(48, 48, 48); }

QPalette BuildThemePalette() { return BuildPaletteFromShell(theme::Shell()); }

QString BuildAppThemeStylesheet() {
  return BuildShellStylesheet(theme::Shell());
}

QString BuildPanelsMenuStylesheet() {
  return BuildPanelsMenuCheckboxIndicatorQss(
      theme::Shell(), QStringLiteral("QMenu"), PreferDarkPanelsMenu());
}

QColor PanelsMenuIconInk() { return theme::Shell().secondary_text; }

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
  // Flat opaque popup — no translucent window plate over the desktop.
  menu->setAttribute(Qt::WA_TranslucentBackground, false);
  menu->setAttribute(Qt::WA_NoSystemBackground, false);
  menu->setAutoFillBackground(true);
  menu->setWindowFlag(Qt::FramelessWindowHint, false);
  menu->setWindowFlag(Qt::NoDropShadowWindowHint, true);
  {
    const theme::ShellTokens shell = theme::Shell();
    QPalette pal = menu->palette();
    pal.setBrush(QPalette::Base, shell.panel_surface);
    pal.setBrush(QPalette::Window, shell.panel_surface);
    menu->setPalette(pal);
  }
  QFont font = menu->font();
  font.setFamilies(
      {QStringLiteral(".AppleSystemUIFont"), QStringLiteral("SF Pro Text"),
       QStringLiteral("Helvetica Neue"), QStringLiteral("Inter"),
       QStringLiteral("Liberation Sans"), QStringLiteral("Sans Serif")});
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
    const int leading = panels
                            ? (12 + 16 + 8 + 14 + 10)  // pad+icon+gap+check+gap
                            : (12 + 16 + 10);          // pad+icon+gap
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
        max_shortcut =
            qMax(max_shortcut, measure(fm, shortcut) + kShortcutSafety);
      }
      if (action->menu() != nullptr) {
        need_submenu = true;
      }
    }
    menu->setProperty("panelsMaxLabelW", max_label);
    menu->setProperty("glassMaxShortcutW", max_shortcut);
    const int hmargin =
        2 * menu->style()->pixelMetric(QStyle::PM_MenuHMargin, nullptr, menu);
    const int shortcut_w = max_shortcut > 0 ? (kShortcutGap + max_shortcut) : 0;
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

void EnsureSystemColorSchemeWatch(QApplication& app) {
  static bool wired = false;
  if (wired) {
    return;
  }
  wired = true;
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
  if (QStyleHints* hints = QGuiApplication::styleHints()) {
    QObject::connect(hints, &QStyleHints::colorSchemeChanged, &app,
                     [&app](Qt::ColorScheme) {
                       if (theme::CurrentShellAppearance() !=
                           theme::ShellAppearance::kSystem) {
                         return;
                       }
                       RefreshAppThemeChrome(app);
                     });
  }
#else
  Q_UNUSED(app);
#endif
}

void ApplyAppTheme(QApplication& app) {
  ApplyAppFont(app);
  EnsureSystemColorSchemeWatch(app);
  QStyle* fusion = QStyleFactory::create(QStringLiteral("Fusion"));
  auto* proxy = new ProxyStyle(fusion);
  proxy->setParent(&app);

  const theme::ShellTokens theme = theme::Shell();
  proxy->setAccentColor(theme.accent);
  proxy->setSelectionBackground(theme.selection_bg);
  proxy->setHoverBackground(theme.hover_bg);
  proxy->setMenuShortcutColor(theme.secondary_text);
  app.setStyle(proxy);
  app.setPalette(BuildPaletteFromShell(theme));
  widget::InvalidateThemeTokens();
  // Do not install the full theme.qss on QApplication. Under Qt 6.2 +
  // WA_NativeWindow Ogre + live Autolink, QCss::Selector::pseudoElement /
  // getOpaqueChildren SEGV during the first paints after RenderSystem ready.
  // Shell chrome uses Fusion+ProxyStyle+palette; panels keep local sheets.
  // Keep a minimal opaque QToolTip sheet so hints stay readable when widgets
  // have no local stylesheet (or inherit transparent tool-button fills).
  app.setStyleSheet(style::toolTipSheet());
}

void RefreshAppThemeChrome(QApplication& app) {
  widget::InvalidateThemeTokens();
  const theme::ShellTokens shell = theme::Shell();
  const QPalette app_pal = BuildPaletteFromShell(shell);
  app.setPalette(app_pal);
  app.setStyleSheet(style::toolTipSheet());
  if (auto* proxy = qobject_cast<ProxyStyle*>(app.style())) {
    proxy->setAccentColor(shell.accent);
    proxy->setSelectionBackground(shell.selection_bg);
    proxy->setHoverBackground(shell.hover_bg);
    proxy->setMenuShortcutColor(shell.secondary_text);
  }

  auto restyle = [&](QWidget* widget) {
    if (widget == nullptr) {
      return;
    }
    const QString name = widget->objectName();
    if (name == QLatin1String(AppThemeIds::kPanelToolbar)) {
      if (auto* frame = qobject_cast<QFrame*>(widget)) {
        ApplyPanelToolbarChrome(frame);
      }
    } else if (name == QLatin1String(AppThemeIds::kPanelFooter)) {
      if (auto* frame = qobject_cast<QFrame*>(widget)) {
        ApplyPanelFooterChrome(frame);
      }
    } else if (name == QLatin1String(AppThemeIds::kPanelTitleTools)) {
      ApplyPanelTitleToolsChrome(widget);
    } else if (name == QLatin1String(AppThemeIds::kMenuBar) ||
               qobject_cast<QMenuBar*>(widget) != nullptr) {
      QPalette pal = app_pal;
      pal.setColor(QPalette::WindowText, shell.window_text);
      pal.setColor(QPalette::ButtonText, shell.window_text);
      pal.setColor(QPalette::Text, shell.text);
      widget->setPalette(pal);
      widget->setStyleSheet(theme::ShellMenuBarStyleSheet() +
                            style::toolTipSheet());
    } else if (name == QLatin1String(AppThemeIds::kToolBar) ||
               qobject_cast<QToolBar*>(widget) != nullptr) {
      QPalette pal = app_pal;
      pal.setColor(QPalette::WindowText, shell.secondary_text);
      pal.setColor(QPalette::ButtonText, shell.secondary_text);
      pal.setColor(QPalette::Text, shell.text);
      widget->setPalette(pal);
      widget->setStyleSheet(theme::ShellToolBarStyleSheet(false) +
                            style::toolTipSheet());
    } else if (qobject_cast<QStatusBar*>(widget) != nullptr) {
      widget->setStyleSheet(theme::ShellStatusBarStyleSheet() +
                            style::toolTipSheet());
    } else if (name == QLatin1String(AppThemeIds::kPanelContent) ||
               name.endsWith(QLatin1String("Panel")) ||
               name.contains(QLatin1String("/"))) {
      // Re-token panel QSS so Liquid / Dark / Light backgrounds refresh.
      if (!ShouldSkipPanelShellBackground(widget)) {
        ApplyPanelShell(widget);
      }
    } else if (qobject_cast<QAbstractItemView*>(widget) != nullptr) {
      ApplyItemViewShellPalette(widget);
    } else if (widget->property("autovizPanelButton").isValid()) {
      if (auto* button = qobject_cast<QPushButton*>(widget)) {
        RefreshPanelButtonStyle(button);
      }
    } else if (name == QLatin1String("DisplaysFrostedFooter")) {
      widget->setStyleSheet(style::sheet(QStringLiteral("displays")));
    } else if (name == QLatin1String("ViewsFrostedFooter")) {
      widget->setStyleSheet(
          style::sheet(QStringLiteral("views"), style::tokens()));
    } else if (qobject_cast<QCheckBox*>(widget) != nullptr ||
               qobject_cast<QRadioButton*>(widget) != nullptr) {
      widget->setPalette(app_pal);
    }
    widget->style()->unpolish(widget);
    widget->style()->polish(widget);
    widget->update();
  };

  for (QWidget* top : app.topLevelWidgets()) {
    if (top == nullptr) {
      continue;
    }
    top->setPalette(app_pal);
    restyle(top);
    const auto widgets = top->findChildren<QWidget*>();
    for (QWidget* child : widgets) {
      restyle(child);
    }
    // Force checkbox / radio indicators to repaint with Shell accent.
    for (QAbstractButton* button : top->findChildren<QAbstractButton*>()) {
      if (qobject_cast<QCheckBox*>(button) != nullptr ||
          qobject_cast<QRadioButton*>(button) != nullptr) {
        button->update();
      }
    }
    top->update();
  }
}

}  // namespace autoviz

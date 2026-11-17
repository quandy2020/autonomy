/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/theme/tokens.hpp"

#include <QGuiApplication>
#include <QPainter>
#include <QPainterPath>
#include <QPalette>
#include <QPen>
#include <QStyleHints>
#include <QtGlobal>

namespace autoviz {
namespace theme {
namespace {

ShellAppearance g_appearance = ShellAppearance::kDark;
QColor g_accent_override;  // invalid ⇒ use defaults

bool SystemPrefersDark() {
  if (QGuiApplication::instance() == nullptr) {
    return false;
  }
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
  if (QGuiApplication::styleHints() != nullptr) {
    return QGuiApplication::styleHints()->colorScheme() ==
           Qt::ColorScheme::Dark;
  }
#endif
  return qGray(QGuiApplication::palette().color(QPalette::Window).rgb()) < 128;
}

void SyncLegacyGlassAliases(ShellTokens& tokens) {
  tokens.glass_fill = tokens.panel_bg;
  tokens.glass_fill_deep = tokens.panel_surface;
  tokens.glass_cool = tokens.panel_cool;
  tokens.glass_sheen = tokens.panel_sheen;
  tokens.glass_rim = tokens.panel_border;
}

void ApplyAccentOverride(ShellTokens& tokens, bool dark) {
  if (!g_accent_override.isValid()) {
    return;
  }
  QColor accent = g_accent_override;
  accent.setAlpha(255);
  tokens.accent = accent;
  tokens.accent_pressed = accent.darker(118);
  tokens.link = accent.darker(110);
  tokens.selection_text = accent;
  if (dark) {
    tokens.accent_soft = QColor(0x3A, 0x3A, 0x3C);
    tokens.selection_bg = QColor(0x3A, 0x3A, 0x3C);
  } else {
    tokens.accent_soft =
        QColor((accent.red() + 255 * 3) / 4, (accent.green() + 255 * 3) / 4,
               (accent.blue() + 255 * 3) / 4);
    tokens.selection_bg = tokens.accent_soft;
  }
}

ShellTokens DarkFlatShell() {
  ShellTokens tokens;
  tokens.window = QColor(0x1C, 0x1C, 0x1E);
  tokens.window_text = QColor(0xF2, 0xF2, 0xF7);
  tokens.base = QColor(0x2C, 0x2C, 0x2E);
  tokens.alternate_base = QColor(0x1C, 0x1C, 0x1E);
  tokens.card = QColor(0x2C, 0x2C, 0x2E);
  tokens.text = QColor(0xF2, 0xF2, 0xF7);
  tokens.secondary_text = QColor(0xAE, 0xAE, 0xB2);
  tokens.button = QColor(0x3A, 0x3A, 0x3C);
  tokens.button_text = QColor(0xF2, 0xF2, 0xF7);
  tokens.accent = QColor(0xA8, 0xC9, 0x4A);
  tokens.accent_pressed = QColor(0x8F, 0xB0, 0x3A);
  tokens.accent_soft = QColor(0x3A, 0x3A, 0x3C);
  tokens.selection_bg = QColor(0x3A, 0x3A, 0x3C);
  tokens.selection_text = QColor(0xA8, 0xC9, 0x4A);
  tokens.link = QColor(0xA8, 0xC9, 0x4A);
  tokens.border = QColor(0x48, 0x48, 0x4A);
  tokens.border_light = QColor(0x38, 0x38, 0x3A);
  tokens.tooltip_base = QColor(0x2C, 0x2C, 0x2E);
  tokens.tooltip_text = QColor(0xF2, 0xF2, 0xF7);
  tokens.disabled_text = QColor(0x63, 0x63, 0x66);
  tokens.toolbar_bg = QColor(0x1C, 0x1C, 0x1E);
  tokens.hover_bg = QColor(0x3A, 0x3A, 0x3C);
  tokens.dock_title = QColor(0x1C, 0x1C, 0x1E);
  tokens.status_bar = QColor(0x1C, 0x1C, 0x1E);
  tokens.panel_bg = QColor(0x2C, 0x2C, 0x2E);
  tokens.panel_surface = QColor(0x1C, 0x1C, 0x1E);
  tokens.panel_cool = QColor(0x2C, 0x2C, 0x2E);
  tokens.panel_sheen = QColor(0x2C, 0x2C, 0x2E);
  tokens.panel_border = QColor(0x48, 0x48, 0x4A);
  tokens.panel_radius = 8;
  tokens.title_radius = 6;
  tokens.control_radius = 6;
  SyncLegacyGlassAliases(tokens);
  return tokens;
}

ShellTokens LightFlatShell() {
  ShellTokens tokens;
  tokens.window = QColor(0xF2, 0xF2, 0xF7);
  tokens.window_text = QColor(0x1C, 0x1C, 0x1E);
  tokens.base = QColor(0xFF, 0xFF, 0xFF);
  tokens.alternate_base = QColor(0xF2, 0xF2, 0xF7);
  tokens.card = QColor(0xFF, 0xFF, 0xFF);
  tokens.text = QColor(0x1C, 0x1C, 0x1E);
  tokens.secondary_text = QColor(0x8E, 0x8E, 0x93);
  tokens.button = QColor(0xE5, 0xE5, 0xEA);
  tokens.button_text = QColor(0x1C, 0x1C, 0x1E);
  tokens.accent = QColor(0xA8, 0xC9, 0x4A);
  tokens.accent_pressed = QColor(0x8F, 0xB0, 0x3A);
  tokens.accent_soft = QColor(0xE2, 0xE2, 0xE6);
  tokens.selection_bg = QColor(0xE2, 0xE2, 0xE6);
  tokens.selection_text = QColor(0x5B, 0x7A, 0x1E);
  tokens.link = QColor(0x5B, 0x7A, 0x1E);
  tokens.border = QColor(0xC6, 0xC6, 0xC8);
  tokens.border_light = QColor(0xE5, 0xE5, 0xEA);
  tokens.tooltip_base = QColor(0xFF, 0xFF, 0xFF);
  tokens.tooltip_text = QColor(0x1C, 0x1C, 0x1E);
  tokens.disabled_text = QColor(0x8E, 0x8E, 0x93);
  tokens.toolbar_bg = QColor(0xF2, 0xF2, 0xF7);
  tokens.hover_bg = QColor(0xE8, 0xE8, 0xEC);
  tokens.dock_title = QColor(0xF2, 0xF2, 0xF7);
  tokens.status_bar = QColor(0xF2, 0xF2, 0xF7);
  tokens.panel_bg = QColor(0xF5, 0xF5, 0xF7);
  tokens.panel_surface = QColor(0xFF, 0xFF, 0xFF);
  tokens.panel_cool = QColor(0xF2, 0xF2, 0xF7);
  tokens.panel_sheen = QColor(0xFF, 0xFF, 0xFF);
  tokens.panel_border = QColor(0xC6, 0xC6, 0xC8);
  tokens.panel_radius = 8;
  tokens.title_radius = 6;
  tokens.control_radius = 6;
  SyncLegacyGlassAliases(tokens);
  return tokens;
}

}  // namespace

void SetShellAppearance(ShellAppearance appearance) {
  if (appearance == ShellAppearance::kLiquid) {
    appearance = ShellAppearance::kLight;
  }
  g_appearance = appearance;
}

ShellAppearance CurrentShellAppearance() { return g_appearance; }

bool ShellIsDark() {
  switch (g_appearance) {
    case ShellAppearance::kDark:
      return true;
    case ShellAppearance::kLight:
    case ShellAppearance::kLiquid:
      return false;
    case ShellAppearance::kSystem:
      return SystemPrefersDark();
  }
  return false;
}

void SetAccentOverride(const QColor& accent) {
  g_accent_override = accent.isValid() ? accent : QColor();
}

QColor AccentOverride() { return g_accent_override; }

OverlayTokens Overlay() { return OverlayTokens{}; }

ShellTokens Shell() {
  const bool dark = ShellIsDark();
  ShellTokens tokens = dark ? DarkFlatShell() : LightFlatShell();
  ApplyAccentOverride(tokens, dark);
  SyncLegacyGlassAliases(tokens);
  return tokens;
}

QString Hex(const QColor& c) { return c.name(QColor::HexRgb); }

QString Rgba(const QColor& c) {
  return QStringLiteral("rgba(%1,%2,%3,%4)")
      .arg(c.red())
      .arg(c.green())
      .arg(c.blue())
      .arg(c.alpha());
}

QString ShellBarCss() { return Hex(Shell().panel_surface); }

QString ShellWindowWashCss() { return Hex(Shell().window); }

QString ShellCssBg() { return Hex(Shell().panel_bg); }

QString ShellCssSurface() { return Hex(Shell().panel_surface); }

QString ShellCssBorder() { return Hex(Shell().panel_border); }

QString ShellCssText() { return Hex(Shell().text); }

QString ShellCssMuted() { return Hex(Shell().secondary_text); }

QString ShellCssAccent() { return Hex(Shell().accent); }

QString ShellCssChromePlate() {
  QColor plate = Shell().toolbar_bg;
  plate.setAlpha(255);
  return Hex(plate);
}

QString ShellMenuBarStyleSheet() {
  const ShellTokens t = Shell();
  const QString plate = ShellCssChromePlate();
  const QString text = Hex(t.window_text);
  const QString rim = Hex(t.panel_border);
  QColor hover = t.hover_bg;
  hover.setAlpha(255);
  return QStringLiteral(
             "QMenuBar { background: %1; color: %2; "
             "border-bottom: 1px solid %3; }"
             "QMenuBar::item { background: transparent; color: %2; "
             "padding: 4px 10px; }"
             "QMenuBar::item:selected { background: %4; "
             "border-radius: 6px; }")
      .arg(plate, text, rim, Hex(hover));
}

QString ShellToolBarStyleSheet(bool object_scoped) {
  const ShellTokens t = Shell();
  const QString plate = ShellCssChromePlate();
  const QString muted = Hex(t.secondary_text);
  const QString rim = Hex(t.panel_border);
  const QString accent = Hex(t.accent);
  QColor hover = t.hover_bg;
  hover.setAlpha(255);
  QColor checked = t.selection_bg;
  checked.setAlpha(255);
  const int radius = t.control_radius;
  const QString root =
      object_scoped ? QStringLiteral("QToolBar#AutovizToolBar")
                    : QStringLiteral("QToolBar#AutovizToolBar, QToolBar");
  const QString btn =
      object_scoped ? QStringLiteral("QToolBar#AutovizToolBar QToolButton")
                    : QStringLiteral("QToolBar QToolButton");
  return QStringLiteral(
             "%1 { background: %2; spacing: 4px; padding: 4px 8px; "
             "border: none; border-bottom: 1px solid %3; }"
             "%4 { color: %5; background: transparent; "
             "border: 1px solid transparent; border-radius: %6px; "
             "padding: 4px 6px; }"
             "%4:hover { background: %7; color: %8; }"
             "%4:checked { background: %9; color: %10; "
             "border-color: transparent; }")
      .arg(root, plate, rim, btn, muted, QString::number(radius))
      .arg(Hex(hover), accent, Hex(checked), accent);
}

QString ShellStatusBarStyleSheet() {
  const ShellTokens t = Shell();
  return QStringLiteral(
             "QStatusBar { background: %1; color: %2; "
             "border-top: 1px solid %3; }")
      .arg(ShellCssChromePlate(), Hex(t.secondary_text), Hex(t.panel_border));
}

void PaintPanelFill(QPainter& painter, const QRectF& rect, qreal radius) {
  const ShellTokens t = Shell();
  painter.setRenderHint(QPainter::Antialiasing, true);
  const QRectF card = rect.adjusted(0.5, 0.5, -0.5, -0.5);
  QPainterPath path;
  path.addRoundedRect(card, radius, radius);
  painter.fillPath(path, t.panel_surface);
  painter.setPen(QPen(t.panel_border, 1.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawPath(path);
}

void PaintShellGlass(QPainter& painter, const QRectF& rect, qreal radius) {
  PaintPanelFill(painter, rect, radius);
}

QColor ShellSelectionFill(int /*alpha*/) {
  QColor fill = Shell().selection_bg;
  if (!fill.isValid() || fill.alpha() == 0) {
    fill = Shell().accent_soft;
  }
  fill.setAlpha(255);
  return fill;
}

QColor ShellHoverFill(int /*alpha*/) {
  QColor fill = Shell().hover_bg;
  fill.setAlpha(255);
  return fill;
}

QColor ShellSelectionInk() {
  return Shell().selection_text.isValid() ? Shell().selection_text
                                         : Shell().accent;
}

}  // namespace theme
}  // namespace autoviz

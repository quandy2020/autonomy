/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/theme/glass.hpp"

#include <QGuiApplication>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QPalette>
#include <QPen>
#include <QtGlobal>

namespace autoviz {
namespace glass {
namespace {

// Default light — avoids solid-black chrome when the desktop is dark.
ShellAppearance g_appearance = ShellAppearance::kLight;
QColor g_accent_override;  // invalid ⇒ use appearance defaults

void ApplyAccentOverride(ShellTokens& tokens) {
  if (!g_accent_override.isValid()) {
    return;
  }
  QColor accent = g_accent_override;
  accent.setAlpha(255);
  tokens.accent = accent;
  tokens.accent_pressed = accent.darker(118);
  tokens.link = accent.darker(110);
  tokens.selection_text = accent;
  // CapCut-style: selected tools keep a neutral gray plate; only ink follows
  // the accent override (not a tinted blue/green wash).
  if (ShellIsDark()) {
    tokens.accent_soft = QColor(0x3A, 0x3A, 0x3C);
    tokens.selection_bg = QColor(0x3A, 0x3A, 0x3C);
  } else {
    tokens.accent_soft = QColor(0xE2, 0xE2, 0xE6);
    tokens.selection_bg = QColor(0xE2, 0xE2, 0xE6);
  }
}

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

ShellTokens DarkShellTokens() {
  ShellTokens tokens;
  // Soft charcoal — not pure #000 — so bars stay readable.
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
  tokens.control_radius = 6;
  tokens.border = QColor(0x48, 0x48, 0x4A);
  tokens.border_light = QColor(0x38, 0x38, 0x3A);
  tokens.tooltip_base = QColor(0x2C, 0x2C, 0x2E);
  tokens.tooltip_text = QColor(0xF2, 0xF2, 0xF7);
  tokens.disabled_text = QColor(0x63, 0x63, 0x66);
  tokens.toolbar_bg = QColor(0x1C, 0x1C, 0x1E);
  tokens.hover_bg = QColor(0x3A, 0x3A, 0x3C);
  tokens.dock_title = QColor(0x1C, 0x1C, 0x1E);
  tokens.status_bar = QColor(0x1C, 0x1C, 0x1E);
  tokens.glass_fill = QColor(0x2C, 0x2C, 0x2E);
  tokens.glass_fill_deep = QColor(0x1C, 0x1C, 0x1E);
  tokens.glass_cool = QColor(0x2C, 0x2C, 0x2E);
  tokens.glass_sheen = QColor(0x2C, 0x2C, 0x2E);
  tokens.glass_rim = QColor(0x48, 0x48, 0x4A);
  return tokens;
}

/**
 * Simulated Liquid Glass (qt-liquid-glass materials on non-macOS):
 * Frosted diffusion + ClearGlass refraction tint + Chromatic rim cues.
 * Tint inspired by Options.tintColor "#80FFFFFF" / Sidebar + Menu materials.
 */
ShellTokens LiquidShellTokens() {
  ShellTokens tokens;
  // Readable frosted plates — keep cool tint but high enough alpha that
  // controls / checkboxes / panel chrome do not wash out on Linux (no real
  // desktop blur behind the window).
  tokens.window = QColor(0xE6, 0xEC, 0xF5);
  tokens.window_text = QColor(0x1A, 0x1A, 0x2E);
  tokens.base = QColor(0xFF, 0xFF, 0xFF);
  tokens.alternate_base = QColor(0xE8, 0xEF, 0xF8);
  tokens.card = QColor(0xFB, 0xFC, 0xFF);
  tokens.text = QColor(0x1A, 0x1A, 0x2E);
  tokens.secondary_text = QColor(0x5A, 0x64, 0x78);
  tokens.button = QColor(0xE4, 0xEB, 0xF6);
  tokens.button_text = QColor(0x1A, 0x1A, 0x2E);
  tokens.accent = QColor(0x5B, 0x7A, 0x1E);
  tokens.accent_pressed = QColor(0x4A, 0x64, 0x18);
  tokens.accent_soft = QColor(0xE2, 0xE2, 0xE6);
  tokens.selection_bg = QColor(0xE2, 0xE2, 0xE6);
  tokens.selection_text = QColor(0x5B, 0x7A, 0x1E);
  tokens.link = QColor(0x5B, 0x7A, 0x1E);
  tokens.border = QColor(0x9A, 0xA8, 0xBC);
  tokens.border_light = QColor(0xC8, 0xD2, 0xE0);
  tokens.tooltip_base = QColor(0xFB, 0xFC, 0xFF);
  tokens.tooltip_text = QColor(0x1A, 0x1A, 0x2E);
  tokens.disabled_text = QColor(0x8E, 0x96, 0xA8);
  tokens.toolbar_bg = QColor(0xEE, 0xF3, 0xFA);
  tokens.hover_bg = QColor(0xD8, 0xE4, 0xF4);
  tokens.dock_title = QColor(0xEE, 0xF3, 0xFA);
  tokens.status_bar = QColor(0xEE, 0xF3, 0xFA);
  // Opaque Liquid chrome — no frosted bleed-through.
  tokens.glass_fill = QColor(0xEE, 0xF3, 0xFA);
  tokens.glass_fill_deep = QColor(0xFB, 0xFC, 0xFF);
  tokens.glass_cool = QColor(0xEE, 0xF3, 0xFA);
  tokens.glass_sheen = QColor(0xFF, 0xFF, 0xFF);
  tokens.glass_rim = QColor(0x9A, 0xA8, 0xBC);
  tokens.panel_radius = 18;
  tokens.title_radius = 14;
  tokens.control_radius = 6;
  return tokens;
}

}  // namespace

void SetShellAppearance(ShellAppearance appearance) {
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

bool ShellIsLiquid() { return g_appearance == ShellAppearance::kLiquid; }

void SetShellAccentOverride(const QColor& accent) {
  g_accent_override = accent.isValid() ? accent : QColor();
}

QColor ShellAccentOverride() { return g_accent_override; }

OverlayTokens Overlay() { return OverlayTokens{}; }

void ForceOpaqueShellMaterials(ShellTokens& tokens) {
  tokens.glass_fill.setAlpha(255);
  tokens.glass_fill_deep.setAlpha(255);
  tokens.glass_cool.setAlpha(255);
  tokens.glass_sheen.setAlpha(255);
  tokens.glass_rim.setAlpha(255);
  tokens.window.setAlpha(255);
  tokens.base.setAlpha(255);
  tokens.card.setAlpha(255);
  tokens.button.setAlpha(255);
  tokens.toolbar_bg.setAlpha(255);
  tokens.dock_title.setAlpha(255);
  tokens.status_bar.setAlpha(255);
  tokens.alternate_base.setAlpha(255);
  tokens.hover_bg.setAlpha(255);
  tokens.border.setAlpha(255);
  tokens.border_light.setAlpha(255);
}

ShellTokens Shell() {
  ShellTokens tokens;
  if (ShellIsLiquid()) {
    tokens = LiquidShellTokens();
  } else if (ShellIsDark()) {
    tokens = DarkShellTokens();
  } else {
    tokens = ShellTokens{};
  }
  ApplyAccentOverride(tokens);
  ForceOpaqueShellMaterials(tokens);
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

QString ShellGlassBarGradientCss() {
  const ShellTokens t = Shell();
  // Flat opaque plate — no translucent gradient wash.
  return Hex(t.glass_fill_deep);
}

QString ShellWindowWashCss() {
  const ShellTokens t = Shell();
  return QStringLiteral(
             "qlineargradient(x1:0,y1:0,x2:1,y2:1,"
             "stop:0 %1, stop:0.45 %2, stop:1 %3)")
      .arg(Hex(t.window), Hex(t.alternate_base), Hex(t.base));
}

QString ShellCssBg() { return Hex(Shell().glass_fill); }

QString ShellCssSurface() { return Hex(Shell().glass_fill_deep); }

QString ShellCssBorder() { return Hex(Shell().glass_rim); }

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
  const QString rim = Rgba(t.glass_rim);
  QColor hover = t.hover_bg;
  if (ShellIsLiquid()) {
    hover = t.accent_soft;
  }
  hover.setAlpha(ShellIsDark() ? 255 : 180);
  const int radius = ShellIsLiquid() ? 8 : 6;
  return QStringLiteral(
             "QMenuBar { background: %1; color: %2; "
             "border-bottom: 1px solid %3; }"
             "QMenuBar::item { background: transparent; color: %2; "
             "padding: 4px 10px; }"
             "QMenuBar::item:selected { background: %4; "
             "border-radius: %5px; }")
      .arg(plate, text, rim, Rgba(hover), QString::number(radius));
}

QString ShellToolBarStyleSheet(bool object_scoped) {
  const ShellTokens t = Shell();
  const QString plate = ShellCssChromePlate();
  const QString muted = Hex(t.secondary_text);
  const QString rim = Rgba(t.glass_rim);
  const QString accent = Hex(t.accent);
  // CapCut: soft gray selected plate + olive ink, no border.
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
  // Chain .arg — multi-arg overload caps at 9 replacements.
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
      .arg(ShellCssChromePlate(), Hex(t.secondary_text), Rgba(t.glass_rim));
}

void PaintShellGlass(QPainter& painter, const QRectF& rect, qreal radius) {
  const ShellTokens t = Shell();
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QRectF card = rect.adjusted(0.5, 0.5, -0.5, -0.5);
  QPainterPath path;
  path.addRoundedRect(card, radius, radius);

  // Solid opaque plate — no frosted / sheen / chromatic washes.
  painter.fillPath(path, t.glass_fill_deep);
  painter.setPen(QPen(t.glass_rim, 1.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawPath(path);
}

QColor ShellSelectionFill(int /*alpha*/) {
  // Opaque soft plate — translucent accent+AA looked like a blurry halo.
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
  // CapCut: selected glyphs use the olive accent on the gray plate.
  return Shell().selection_text.isValid() ? Shell().selection_text
                                         : Shell().accent;
}

}  // namespace glass
}  // namespace autoviz

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
  tokens.accent = QColor(0x0A, 0x84, 0xFF);
  tokens.accent_pressed = QColor(0x00, 0x71, 0xE3);
  tokens.accent_soft = QColor(0x1B, 0x3A, 0x5A);
  tokens.selection_bg = QColor(0x1B, 0x3A, 0x5A);
  tokens.selection_text = QColor(0xF2, 0xF2, 0xF7);
  tokens.link = QColor(0x40, 0x9C, 0xFF);
  tokens.border = QColor(0x48, 0x48, 0x4A);
  tokens.border_light = QColor(0x38, 0x38, 0x3A);
  tokens.tooltip_base = QColor(0x2C, 0x2C, 0x2E);
  tokens.tooltip_text = QColor(0xF2, 0xF2, 0xF7);
  tokens.disabled_text = QColor(0x63, 0x63, 0x66);
  tokens.toolbar_bg = QColor(0x1C, 0x1C, 0x1E);
  tokens.hover_bg = QColor(0x3A, 0x3A, 0x3C);
  tokens.dock_title = QColor(0x1C, 0x1C, 0x1E);
  tokens.status_bar = QColor(0x1C, 0x1C, 0x1E);
  tokens.glass_fill = QColor(44, 44, 46, 224);
  tokens.glass_fill_deep = QColor(28, 28, 30, 245);
  tokens.glass_cool = QColor(10, 132, 255, 18);
  tokens.glass_sheen = QColor(255, 255, 255, 20);
  tokens.glass_rim = QColor(84, 84, 88, 120);
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
  tokens.accent = QColor(0x00, 0x7A, 0xFF);
  tokens.accent_pressed = QColor(0x00, 0x5F, 0xD6);
  tokens.accent_soft = QColor(0xD6, 0xE8, 0xFF);
  tokens.selection_bg = QColor(0xD6, 0xE8, 0xFF);
  tokens.selection_text = QColor(0x1A, 0x1A, 0x2E);
  tokens.link = QColor(0x00, 0x66, 0xCC);
  tokens.border = QColor(0x9A, 0xA8, 0xBC);
  tokens.border_light = QColor(0xC8, 0xD2, 0xE0);
  tokens.tooltip_base = QColor(0xFB, 0xFC, 0xFF);
  tokens.tooltip_text = QColor(0x1A, 0x1A, 0x2E);
  tokens.disabled_text = QColor(0x8E, 0x96, 0xA8);
  tokens.toolbar_bg = QColor(0xEE, 0xF3, 0xFA);
  tokens.hover_bg = QColor(0xD8, 0xE4, 0xF4);
  tokens.dock_title = QColor(0xEE, 0xF3, 0xFA);
  tokens.status_bar = QColor(0xEE, 0xF3, 0xFA);
  // Frosted fill (bright diffusion) + ClearGlass cool wash — mostly opaque.
  tokens.glass_fill = QColor(255, 255, 255, 228);
  tokens.glass_fill_deep = QColor(248, 250, 255, 245);
  tokens.glass_cool = QColor(120, 180, 255, 28);
  tokens.glass_sheen = QColor(255, 255, 255, 180);
  tokens.glass_rim = QColor(170, 185, 205, 160);
  tokens.panel_radius = 18;
  tokens.title_radius = 14;
  tokens.control_radius = 12;
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

OverlayTokens Overlay() { return OverlayTokens{}; }

ShellTokens Shell() {
  if (ShellIsLiquid()) {
    return LiquidShellTokens();
  }
  return ShellIsDark() ? DarkShellTokens() : ShellTokens{};
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
  QColor leading = t.glass_fill_deep;
  leading.setAlpha(qMax(leading.alpha(), 245));
  return QStringLiteral(
             "qlineargradient(x1:0,y1:0,x2:0,y2:1,"
             "stop:0 %1, stop:0.35 %2, stop:1 %3)")
      .arg(Rgba(leading), Rgba(t.glass_fill_deep), Rgba(t.glass_fill));
}

QString ShellWindowWashCss() {
  const ShellTokens t = Shell();
  return QStringLiteral(
             "qlineargradient(x1:0,y1:0,x2:1,y2:1,"
             "stop:0 %1, stop:0.45 %2, stop:1 %3)")
      .arg(Hex(t.window), Hex(t.alternate_base), Hex(t.base));
}

QString ShellCssBg() { return Rgba(Shell().glass_fill); }

QString ShellCssSurface() { return Rgba(Shell().glass_fill_deep); }

QString ShellCssBorder() { return Rgba(Shell().glass_rim); }

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
  const QString accent_pressed = Hex(t.accent_pressed);
  QColor hover = ShellIsLiquid() ? t.accent_soft : t.hover_bg;
  hover.setAlpha(ShellIsDark() ? 255 : 160);
  QColor checked = t.accent_soft;
  checked.setAlpha(ShellIsDark() ? 255 : 200);
  const int radius = ShellIsLiquid() ? 8 : 5;
  const QString root =
      object_scoped ? QStringLiteral("QToolBar#AutovizToolBar")
                    : QStringLiteral("QToolBar#AutovizToolBar, QToolBar");
  const QString btn =
      object_scoped ? QStringLiteral("QToolBar#AutovizToolBar QToolButton")
                    : QStringLiteral("QToolBar QToolButton");
  // Chain .arg — multi-arg overload caps at 9 replacements.
  return QStringLiteral(
             "%1 { background: %2; spacing: 2px; padding: 3px 8px; "
             "border: none; border-bottom: 1px solid %3; }"
             "%4 { color: %5; background: transparent; "
             "border: 1px solid transparent; border-radius: %6px; "
             "padding: 3px 6px; }"
             "%4:hover { background: %7; color: %8; }"
             "%4:checked { background: %9; color: %10; }")
      .arg(root, plate, rim, btn, muted, QString::number(radius))
      .arg(Rgba(hover), accent, Rgba(checked), accent_pressed);
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
  const bool liquid = ShellIsLiquid();
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QRectF card = rect.adjusted(0.5, 0.5, -0.5, -0.5);
  QPainterPath path;
  path.addRoundedRect(card, radius, radius);

  painter.fillPath(path, t.glass_fill_deep);

  QColor body_end = t.glass_fill;
  body_end.setAlpha(qMin(body_end.alpha(), liquid ? 90 : 120));
  QLinearGradient body(card.topLeft(), card.bottomLeft());
  body.setColorAt(0.0, t.glass_fill);
  body.setColorAt(1.0, body_end);
  painter.fillPath(path, body);

  painter.fillPath(path, t.glass_cool);

  if (liquid) {
    // ClearGlass-style cool refraction wash (diagonal).
    QColor cool_b = QColor(180, 220, 255, 28);
    QLinearGradient cool(card.topRight(), card.bottomLeft());
    cool.setColorAt(0.0, cool_b);
    cool.setColorAt(0.55, QColor(255, 255, 255, 0));
    cool.setColorAt(1.0, QColor(255, 200, 230, 18));
    painter.fillPath(path, cool);
  }

  QColor sheen_end = t.glass_sheen;
  sheen_end.setAlpha(0);
  QLinearGradient sheen(
      card.topLeft(),
      QPointF(card.left(), card.top() + card.height() * (liquid ? 0.55 : 0.45)));
  sheen.setColorAt(0.0, t.glass_sheen);
  sheen.setColorAt(1.0, sheen_end);
  painter.fillPath(path, sheen);

  if (liquid) {
    // Chromatic aberration rim (qt-liquid-glass Material::Chromatic cue).
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(QColor(255, 90, 120, 42), 1.0));
    painter.drawRoundedRect(card.adjusted(1.0, 1.0, -0.5, -0.5), radius,
                            radius);
    painter.setPen(QPen(QColor(80, 180, 255, 48), 1.0));
    painter.drawRoundedRect(card.adjusted(0.5, 0.5, -1.0, -1.0), radius,
                            radius);
  }

  painter.setPen(QPen(t.glass_rim, 1.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawPath(path);
}

QColor ShellSelectionFill(int alpha) {
  QColor fill = Shell().accent;
  fill.setAlpha(qBound(0, alpha, 255));
  return fill;
}

QColor ShellHoverFill(int alpha) {
  QColor fill = ShellIsLiquid() ? Shell().accent_soft : Shell().hover_bg;
  if (fill.alpha() == 255) {
    fill.setAlpha(qBound(0, alpha, 255));
  } else {
    fill.setAlpha(qBound(0, qMin(fill.alpha(), alpha), 255));
  }
  return fill;
}

QColor ShellSelectionInk() {
  // Prefer pressed accent (darker) so selected labels stay readable on soft fill.
  return Shell().accent_pressed;
}

}  // namespace glass
}  // namespace autoviz

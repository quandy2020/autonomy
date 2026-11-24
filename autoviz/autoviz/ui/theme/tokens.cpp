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

ShellAppearance g_appearance = ShellAppearance::kLight;
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
  tokens.window = QColor(0x1B, 0x22, 0x30);
  tokens.window_text = QColor(0xE7, 0xED, 0xF6);
  tokens.base = QColor(0x23, 0x2C, 0x3C);
  tokens.alternate_base = QColor(0x1B, 0x22, 0x30);
  tokens.card = QColor(0x23, 0x2C, 0x3C);
  tokens.text = QColor(0xE7, 0xED, 0xF6);
  tokens.secondary_text = QColor(0x93, 0xA1, 0xB5);
  tokens.button = QColor(0x2B, 0x35, 0x49);
  tokens.button_text = QColor(0xE7, 0xED, 0xF6);
  tokens.accent = QColor(0x4F, 0xD1, 0xC5);
  tokens.accent_pressed = QColor(0x38, 0xB2, 0xA6);
  tokens.accent_soft = QColor(0x1F, 0x3A, 0x44);
  tokens.selection_bg = QColor(0x26, 0x40, 0x4A);
  tokens.selection_text = QColor(0xA7, 0xF3, 0xEC);
  tokens.link = QColor(0x4F, 0xD1, 0xC5);
  tokens.border = QColor(0x33, 0x40, 0x5A);
  tokens.border_light = QColor(0x2A, 0x34, 0x46);
  tokens.tooltip_base = QColor(0x23, 0x2C, 0x3C);
  tokens.tooltip_text = QColor(0xE7, 0xED, 0xF6);
  tokens.disabled_text = QColor(0x6B, 0x7A, 0x90);
  tokens.danger = QColor(0xF8, 0x71, 0x71);
  tokens.danger_hover = QColor(0xFC, 0xA5, 0xA5);
  tokens.danger_pressed = QColor(0xEF, 0x44, 0x44);
  tokens.danger_soft = QColor(0x3B, 0x1E, 0x22);
  tokens.danger_border = QColor(0x7F, 0x3A, 0x3E);
  tokens.warning = QColor(0xF5, 0x9E, 0x0B);
  tokens.toolbar_bg = QColor(0x1B, 0x22, 0x30);
  tokens.hover_bg = QColor(0x2A, 0x34, 0x46);
  tokens.dock_title = QColor(0x1B, 0x22, 0x30);
  tokens.status_bar = QColor(0x1B, 0x22, 0x30);
  tokens.panel_bg = QColor(0x1E, 0x26, 0x34);
  tokens.panel_surface = QColor(0x23, 0x2C, 0x3C);
  tokens.panel_cool = QColor(0x28, 0x31, 0x42);
  tokens.panel_sheen = QColor(0x2B, 0x35, 0x49);
  tokens.panel_border = QColor(0x33, 0x40, 0x5A);
  tokens.panel_radius = 14;
  tokens.title_radius = 10;
  tokens.control_radius = 10;
  tokens.dock_title_height = 38;
  tokens.field_height = 30;
  tokens.button_height = 32;
  tokens.button_radius = 12;
  tokens.content_margin = 12;
  tokens.content_spacing = 6;
  tokens.section_spacing = 8;
  tokens.gutter = 7;
  SyncLegacyGlassAliases(tokens);
  return tokens;
}

ShellTokens LightFlatShell() {
  ShellTokens tokens;
  tokens.window = QColor(0xED, 0xF0, 0xF5);
  tokens.window_text = QColor(0x10, 0x18, 0x28);
  tokens.base = QColor(0xFF, 0xFF, 0xFF);
  tokens.alternate_base = QColor(0xF4, 0xF6, 0xFA);
  tokens.card = QColor(0xFF, 0xFF, 0xFF);
  tokens.text = QColor(0x10, 0x18, 0x28);
  tokens.secondary_text = QColor(0x66, 0x70, 0x85);
  tokens.button = QColor(0xEE, 0xF1, 0xF6);
  tokens.button_text = QColor(0x10, 0x18, 0x28);
  tokens.accent = QColor(0x0E, 0x93, 0x84);
  tokens.accent_pressed = QColor(0x0B, 0x7A, 0x6E);
  tokens.accent_soft = QColor(0xD7, 0xF0, 0xEC);
  tokens.selection_bg = QColor(0xD7, 0xF0, 0xEC);
  tokens.selection_text = QColor(0x0B, 0x5F, 0x57);
  tokens.link = QColor(0x0E, 0x7C, 0x72);
  tokens.border = QColor(0xD5, 0xDB, 0xE5);
  tokens.border_light = QColor(0xE7, 0xEB, 0xF2);
  tokens.tooltip_base = QColor(0xFF, 0xFF, 0xFF);
  tokens.tooltip_text = QColor(0x10, 0x18, 0x28);
  tokens.disabled_text = QColor(0x98, 0xA2, 0xB3);
  tokens.danger = QColor(0xDC, 0x26, 0x26);
  tokens.danger_hover = QColor(0xB9, 0x1C, 0x1C);
  tokens.danger_pressed = QColor(0x99, 0x1B, 0x1B);
  tokens.danger_soft = QColor(0xFE, 0xE2, 0xE2);
  tokens.danger_border = QColor(0xFC, 0xA5, 0xA5);
  tokens.warning = QColor(0xD9, 0x77, 0x06);
  tokens.toolbar_bg = QColor(0xF4, 0xF6, 0xFA);
  tokens.hover_bg = QColor(0xE9, 0xEE, 0xF5);
  tokens.dock_title = QColor(0xF1, 0xF4, 0xF9);
  tokens.status_bar = QColor(0xF4, 0xF6, 0xFA);
  tokens.panel_bg = QColor(0xF4, 0xF6, 0xFA);
  tokens.panel_surface = QColor(0xFF, 0xFF, 0xFF);
  tokens.panel_cool = QColor(0xEE, 0xF2, 0xF8);
  tokens.panel_sheen = QColor(0xFF, 0xFF, 0xFF);
  tokens.panel_border = QColor(0xDF, 0xE5, 0xEE);
  tokens.panel_radius = 14;
  tokens.title_radius = 10;
  tokens.control_radius = 10;
  tokens.dock_title_height = 38;
  tokens.field_height = 30;
  tokens.button_height = 32;
  tokens.button_radius = 12;
  tokens.content_margin = 12;
  tokens.content_spacing = 6;
  tokens.section_spacing = 8;
  tokens.gutter = 7;
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

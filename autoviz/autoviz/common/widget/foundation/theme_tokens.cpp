/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

#include <QGuiApplication>
#include <QPalette>
#include <QString>
#include <QVariant>
#include <QWidget>
#include <QtMath>

#include "autoviz/ui/theme/glass.hpp"

namespace autoviz::widget {
namespace {

QColor Blend(const QColor& foreground, const QColor& background, int alpha) {
  alpha = qBound(0, alpha, 255);
  const int inverse = 255 - alpha;
  return QColor(
      (foreground.red() * alpha + background.red() * inverse) / 255,
      (foreground.green() * alpha + background.green() * inverse) / 255,
      (foreground.blue() * alpha + background.blue() * inverse) / 255,
      (foreground.alpha() * alpha + background.alpha() * inverse) / 255);
}

QColor CompositeOver(const QColor& foreground, const QColor& background) {
  const qreal alpha = foreground.alphaF();
  if (alpha >= 1.0) {
    return foreground;
  }
  return QColor::fromRgbF(
      foreground.redF() * alpha + background.redF() * (1.0 - alpha),
      foreground.greenF() * alpha + background.greenF() * (1.0 - alpha),
      foreground.blueF() * alpha + background.blueF() * (1.0 - alpha));
}

qreal LinearChannel(qreal channel) {
  return channel <= 0.04045 ? channel / 12.92
                            : qPow((channel + 0.055) / 1.055, 2.4);
}

qreal RelativeLuminance(const QColor& color) {
  return 0.2126 * LinearChannel(color.redF()) +
         0.7152 * LinearChannel(color.greenF()) +
         0.0722 * LinearChannel(color.blueF());
}

qreal ContrastRatio(const QColor& foreground, const QColor& background) {
  const QColor opaque_background = CompositeOver(background, QColor(Qt::white));
  const QColor opaque_foreground = CompositeOver(foreground, opaque_background);
  const qreal foreground_luminance = RelativeLuminance(opaque_foreground);
  const qreal background_luminance = RelativeLuminance(opaque_background);
  const qreal lighter = qMax(foreground_luminance, background_luminance);
  const qreal darker = qMin(foreground_luminance, background_luminance);
  return (lighter + 0.05) / (darker + 0.05);
}

QColor EnsureContrast(const QColor& candidate, const QColor& background,
                      qreal target_ratio) {
  const QColor opaque_candidate = CompositeOver(candidate, background);
  if (ContrastRatio(opaque_candidate, background) >= target_ratio) {
    return opaque_candidate;
  }

  const QColor black(Qt::black);
  const QColor white(Qt::white);
  const QColor endpoint =
      ContrastRatio(black, background) >= ContrastRatio(white, background)
          ? black
          : white;
  int low = 0;
  int high = 255;
  while (low < high) {
    const int midpoint = (low + high) / 2;
    const QColor mixed = Blend(endpoint, opaque_candidate, midpoint);
    if (ContrastRatio(mixed, background) >= target_ratio) {
      high = midpoint;
    } else {
      low = midpoint + 1;
    }
  }
  return Blend(endpoint, opaque_candidate, high);
}

QColor BestForeground(const QColor& preferred, const QColor& background,
                      qreal target_ratio) {
  if (ContrastRatio(preferred, background) >= target_ratio) {
    return CompositeOver(preferred, background);
  }
  const QColor black(Qt::black);
  const QColor white(Qt::white);
  return ContrastRatio(black, background) >= ContrastRatio(white, background)
             ? black
             : white;
}

QColor WithAlpha(QColor color, int alpha) {
  color.setAlpha(qBound(0, alpha, 255));
  return color;
}

ThemeAppearance AppearanceFromColor(const QColor& background) {
  const QColor opaque_background = CompositeOver(background, QColor(Qt::white));
  return qGray(opaque_background.rgb()) < 128 ? ThemeAppearance::kDark
                                              : ThemeAppearance::kLight;
}

ThemeAppearance SystemAppearance() {
  if (QGuiApplication::instance() == nullptr) {
    return ThemeAppearance::kLight;
  }
  return AppearanceFromColor(
      QGuiApplication::palette().color(QPalette::Window));
}

void DeriveDarkAppearance(ThemeTokens* tokens, style::Tone tone) {
  // UIKit dark appearance uses explicit dynamic layers. Deriving dark colors
  // from the light palette produces muddy intermediate grays and loses the
  // clear grouped-background hierarchy.
  tokens->window = QColor(0, 0, 0);
  tokens->surface = QColor(28, 28, 30);
  tokens->surface_raised = QColor(44, 44, 46);
  tokens->text = QColor(242, 242, 247);
  tokens->text_muted = QColor(174, 174, 178);
  tokens->accent = QColor(0xA8, 0xC9, 0x4A);
  tokens->accent_pressed = QColor(0x8F, 0xB0, 0x3A);
  tokens->on_accent = QColor(Qt::black);
  tokens->accent_filled = QColor(0x5B, 0x7A, 0x1E);
  tokens->accent_filled_hovered = QColor(0x4A, 0x64, 0x18);
  tokens->accent_filled_pressed = QColor(0x3D, 0x54, 0x14);
  tokens->on_accent_filled = QColor(Qt::white);
  tokens->border = QColor(72, 72, 74);
  tokens->info = tokens->accent;
  tokens->success = QColor(48, 209, 88);
  tokens->warning = QColor(255, 159, 10);
  tokens->danger = QColor(255, 69, 58);
  tokens->danger_filled = QColor(215, 0, 21);
  tokens->danger_filled_hovered = QColor(204, 0, 20);
  tokens->danger_filled_pressed = QColor(184, 0, 18);
  tokens->on_danger_filled = QColor(Qt::white);
  tokens->disabled = QColor(99, 99, 102);
  if (tone == style::Tone::Frost) {
    tokens->surface = Blend(tokens->accent, tokens->surface, 10);
    tokens->surface_raised = Blend(tokens->accent, tokens->surface_raised, 8);
  }
}

void ResolveSemanticRoles(ThemeTokens* tokens, ThemeAppearance appearance) {
  const QColor themed_accent_pressed = tokens->accent_pressed;
  const QColor opaque_window = CompositeOver(tokens->window, QColor(Qt::white));
  const QColor contrast_surface = CompositeOver(tokens->surface, opaque_window);
  tokens->is_dark = appearance == ThemeAppearance::kDark;
  const QColor polarity =
      tokens->is_dark ? QColor(Qt::white) : QColor(Qt::black);
  const QColor inverse_polarity =
      tokens->is_dark ? QColor(Qt::black) : QColor(Qt::white);

  tokens->text = EnsureContrast(tokens->text, contrast_surface, 4.5);
  tokens->text_secondary =
      EnsureContrast(tokens->text_muted, contrast_surface, 4.5);
  tokens->text_tertiary = EnsureContrast(
      Blend(tokens->text, tokens->surface, tokens->is_dark ? 112 : 96),
      contrast_surface, 3.0);
  tokens->text_muted = tokens->text_secondary;
  tokens->placeholder = tokens->text_secondary;

  if (tokens->is_dark) {
    tokens->surface_sunken = QColor(22, 22, 24);
    tokens->surface_material = QColor(58, 58, 60);
    tokens->surface_hovered = QColor(58, 58, 60);
    tokens->surface_pressed = QColor(72, 72, 74);
  } else {
    tokens->surface_sunken = QColor(242, 242, 247);
    tokens->surface_material = QColor(229, 229, 234);
    tokens->surface_hovered = QColor(220, 220, 224);
    tokens->surface_pressed = QColor(209, 209, 214);
  }
  // CapCut: selected controls use a neutral gray plate, olive ink separately.
  tokens->surface_selected =
      tokens->is_dark ? QColor(0x3A, 0x3A, 0x3C) : QColor(0xE2, 0xE2, 0xE6);
  tokens->accent_text = EnsureContrast(tokens->accent, contrast_surface, 4.5);

  tokens->border_subtle =
      tokens->is_dark ? QColor(56, 56, 58) : QColor(198, 198, 200);
  tokens->border_strong = EnsureContrast(tokens->border, contrast_surface, 3.0);
  tokens->border = tokens->border_subtle;
  tokens->separator = tokens->border_subtle;
  tokens->focus_ring = EnsureContrast(tokens->accent, contrast_surface, 3.0);
  tokens->focus_ring_outer = WithAlpha(tokens->focus_ring, 64);
  tokens->shadow_ambient =
      WithAlpha(QColor(Qt::black), tokens->is_dark ? 128 : 32);
  tokens->scrim = WithAlpha(QColor(Qt::black), tokens->is_dark ? 144 : 104);

  tokens->accent_hovered =
      Blend(polarity, tokens->accent, tokens->is_dark ? 22 : 0);
  if (!tokens->is_dark) {
    tokens->accent_hovered = Blend(inverse_polarity, tokens->accent, 14);
  }
  tokens->accent_pressed =
      themed_accent_pressed.isValid()
          ? themed_accent_pressed
          : Blend(inverse_polarity, tokens->accent, tokens->is_dark ? 20 : 28);
  tokens->on_accent = BestForeground(tokens->on_accent, tokens->accent, 4.5);
  tokens->on_accent_hovered =
      BestForeground(tokens->on_accent, tokens->accent_hovered, 4.5);
  tokens->on_accent_pressed =
      BestForeground(tokens->on_accent, tokens->accent_pressed, 4.5);

  tokens->on_accent_filled =
      BestForeground(tokens->on_accent_filled, tokens->accent_filled, 4.5);

  tokens->danger_hovered =
      Blend(tokens->is_dark ? QColor(Qt::white) : QColor(Qt::black),
            tokens->danger, tokens->is_dark ? 20 : 14);
  tokens->danger_pressed =
      Blend(QColor(Qt::black), tokens->danger, tokens->is_dark ? 18 : 28);
  tokens->on_danger = BestForeground(QColor(Qt::white), tokens->danger, 4.5);
  tokens->on_danger_hovered =
      BestForeground(tokens->on_danger, tokens->danger_hovered, 4.5);
  tokens->on_danger_pressed =
      BestForeground(tokens->on_danger, tokens->danger_pressed, 4.5);
  tokens->on_danger_filled =
      BestForeground(tokens->on_danger_filled, tokens->danger_filled, 4.5);
  tokens->disabled = Blend(tokens->text, tokens->surface, 74);
}

void SeedFromShell(ThemeTokens* tokens, const glass::ShellTokens& shell,
                   style::Tone tone) {
  tokens->window = shell.window;
  tokens->surface = shell.card;
  tokens->surface_raised = shell.base;
  tokens->text = shell.text;
  tokens->text_muted = shell.secondary_text;
  tokens->accent = shell.accent;
  tokens->accent_pressed = shell.accent_pressed;
  tokens->on_accent = shell.selection_text;
  tokens->accent_filled = shell.accent;
  tokens->accent_filled_hovered = shell.accent_pressed;
  tokens->accent_filled_pressed = shell.accent_pressed.darker(110);
  tokens->on_accent_filled = QColor(Qt::white);
  tokens->surface_selected = shell.selection_bg;
  tokens->border = shell.border;
  tokens->info = shell.accent;
  tokens->success = QColor(52, 199, 89);
  tokens->warning = QColor(255, 149, 0);
  tokens->danger = QColor(255, 59, 48);
  tokens->danger_filled = QColor(215, 0, 21);
  tokens->danger_filled_hovered = QColor(204, 0, 20);
  tokens->danger_filled_pressed = QColor(184, 0, 18);
  tokens->on_danger_filled = QColor(Qt::white);
  tokens->disabled = shell.disabled_text;
  if (tone == style::Tone::Frost) {
    tokens->surface = Blend(tokens->accent, tokens->surface, 5);
    tokens->surface_raised = Blend(tokens->accent, tokens->surface_raised, 3);
  }
  if (glass::ShellIsLiquid()) {
    tokens->radius_small = qMax(8, shell.control_radius - 2);
    tokens->radius_medium = shell.control_radius;
    tokens->radius_large = shell.panel_radius;
  }
}

ThemeTokens BuildTokens(style::Tone tone, ThemeAppearance appearance) {
  ThemeTokens result;
  // Bridge Autoviz widget kit ↔ glass Shell (Liquid / Light / Dark) so every
  // self-painted control follows qt-liquid-glass chrome when selected.
  if (appearance == ThemeAppearance::kDark) {
    SeedFromShell(&result, glass::Shell(), tone);
    // When the live shell is Liquid/Light, still synthesize a dark kit for
    // explicitly dark subtrees (tests / nested dark hosts).
    if (!glass::ShellIsDark()) {
      DeriveDarkAppearance(&result, tone);
    }
  } else if (glass::ShellIsLiquid() || !glass::ShellIsDark()) {
    SeedFromShell(&result, glass::Shell(), tone);
  } else {
    // Live shell is dark but caller asked for light — use default light plate.
    SeedFromShell(&result, glass::ShellTokens{}, tone);
  }
  ResolveSemanticRoles(&result, appearance);
  return result;
}

struct ThemeTokenCache {
  ThemeTokens shell_light;
  ThemeTokens shell_dark;
  ThemeTokens frost_light;
  ThemeTokens frost_dark;
  bool valid = false;
};

ThemeTokenCache& Cache() {
  static ThemeTokenCache cache;
  return cache;
}

}  // namespace

const ThemeTokens& GetThemeTokens(style::Tone tone,
                                  ThemeAppearance appearance) {
  ThemeTokenCache& cache = Cache();
  if (!cache.valid) {
    cache.shell_light =
        BuildTokens(style::Tone::Shell, ThemeAppearance::kLight);
    cache.shell_dark = BuildTokens(style::Tone::Shell, ThemeAppearance::kDark);
    cache.frost_light =
        BuildTokens(style::Tone::Frost, ThemeAppearance::kLight);
    cache.frost_dark = BuildTokens(style::Tone::Frost, ThemeAppearance::kDark);
    cache.valid = true;
  }
  const ThemeAppearance resolved =
      appearance == ThemeAppearance::kSystem ? SystemAppearance() : appearance;
  if (tone == style::Tone::Frost) {
    return resolved == ThemeAppearance::kDark ? cache.frost_dark
                                              : cache.frost_light;
  }
  return resolved == ThemeAppearance::kDark ? cache.shell_dark
                                            : cache.shell_light;
}

const ThemeTokens& GetThemeTokens(const QWidget* widget, style::Tone tone) {
  if (widget == nullptr) {
    return GetThemeTokens(tone);
  }

  const QString appearance =
      widget->property("autovizAppearance").toString().toLower();
  if (appearance == QLatin1String("dark")) {
    return GetThemeTokens(tone, ThemeAppearance::kDark);
  }
  if (appearance == QLatin1String("light")) {
    return GetThemeTokens(tone, ThemeAppearance::kLight);
  }
  return GetThemeTokens(
      tone,
      AppearanceFromColor(widget->palette().color(widget->backgroundRole())));
}

void InvalidateThemeTokens() noexcept { Cache().valid = false; }

}  // namespace autoviz::widget

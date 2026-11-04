/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>

#include "autoviz/ui/theme/style.hpp"

class QWidget;

namespace autoviz::widget {

// Appearance is independent from Tone: Tone selects the product surface
// family, while appearance selects its light/dark semantic resolution.
enum class ThemeAppearance { kSystem, kLight, kDark };

// Semantic values shared by all Autoviz widgets. Component code must consume
// these roles instead of embedding colors or geometry constants.
struct ThemeTokens final {
  QColor window;
  QColor surface;
  QColor surface_sunken;
  QColor surface_raised;
  QColor surface_material;
  QColor surface_hovered;
  QColor surface_pressed;
  QColor surface_selected;
  QColor text;
  QColor text_secondary;
  QColor text_tertiary;
  QColor text_muted;
  QColor placeholder;
  QColor accent;
  QColor accent_text;
  QColor accent_hovered;
  QColor accent_pressed;
  QColor on_accent;
  QColor accent_filled;
  QColor accent_filled_hovered;
  QColor accent_filled_pressed;
  QColor on_accent_filled;
  QColor on_accent_hovered;
  QColor on_accent_pressed;
  QColor border;
  QColor border_subtle;
  QColor border_strong;
  QColor separator;
  QColor focus_ring;
  QColor focus_ring_outer;
  QColor shadow_ambient;
  QColor scrim;
  QColor info;
  QColor success;
  QColor warning;
  QColor danger;
  QColor danger_hovered;
  QColor danger_pressed;
  QColor danger_filled;
  QColor danger_filled_hovered;
  QColor danger_filled_pressed;
  QColor on_danger_filled;
  QColor on_danger;
  QColor on_danger_hovered;
  QColor on_danger_pressed;
  QColor disabled;

  int radius_small = 8;
  int radius_medium = 12;
  int radius_large = 16;
  int spacing_xs = 4;
  int spacing_sm = 8;
  int spacing_md = 12;
  int spacing_lg = 16;
  int control_height_compact = 28;
  int control_height_small = 32;
  int control_height_medium = 44;
  int control_height_large = 50;
  int focus_ring_width = 3;
  int animation_fast_ms = 120;
  int animation_normal_ms = 200;
  bool is_dark = false;
};

// Returns tokens bridged from the existing Autoviz style system. References
// remain valid, but callers should not retain one across a theme change.
const ThemeTokens& GetThemeTokens(
    style::Tone tone = style::Tone::Shell,
    ThemeAppearance appearance = ThemeAppearance::kSystem);

// Resolves appearance from a themed widget before falling back to its
// inherited palette. Self-painted controls must use this overload so an
// explicitly light or dark subtree does not accidentally use the process-wide
// application appearance.
const ThemeTokens& GetThemeTokens(const QWidget* widget,
                                  style::Tone tone = style::Tone::Shell);

// Invalidates the GUI-thread cache after the application changes its theme.
void InvalidateThemeTokens() noexcept;

}  // namespace autoviz::widget

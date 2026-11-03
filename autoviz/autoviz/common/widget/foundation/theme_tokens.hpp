/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>

#include "autoviz/ui/theme/style.hpp"

namespace autoviz::widget {

// Semantic values shared by all Autoviz widgets. Component code must consume
// these roles instead of embedding colors or geometry constants.
struct ThemeTokens final {
  QColor window;
  QColor surface;
  QColor surface_raised;
  QColor surface_hovered;
  QColor text;
  QColor text_muted;
  QColor accent;
  QColor accent_hovered;
  QColor accent_pressed;
  QColor on_accent;
  QColor border;
  QColor focus_ring;
  QColor success;
  QColor warning;
  QColor danger;
  QColor disabled;

  int radius_small = 4;
  int radius_medium = 8;
  int radius_large = 12;
  int spacing_xs = 4;
  int spacing_sm = 8;
  int spacing_md = 12;
  int spacing_lg = 16;
  int control_height_compact = 24;
  int control_height_small = 28;
  int control_height_medium = 36;
  int control_height_large = 44;
  int focus_ring_width = 2;
  int animation_fast_ms = 120;
  int animation_normal_ms = 180;
};

// Returns tokens bridged from the existing Autoviz style system. References
// remain valid, but callers should not retain one across a theme change.
const ThemeTokens& GetThemeTokens(style::Tone tone = style::Tone::Shell);

// Invalidates the GUI-thread cache after the application changes its theme.
void InvalidateThemeTokens() noexcept;

}  // namespace autoviz::widget

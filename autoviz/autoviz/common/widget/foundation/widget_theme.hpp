/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QPalette>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QWidget;

namespace autoviz::widget {

// Builds a Qt palette from the semantic Autoviz tokens. Active, inactive, and
// disabled groups are all resolved so native model/view, editor, date/time,
// slider, and combo widgets retain host-platform behavior in either appearance.
QPalette BuildWidgetPalette(
    style::Tone tone = style::Tone::Shell, const QPalette& base = QPalette(),
    ThemeAppearance appearance = ThemeAppearance::kSystem);

// Applies the semantic palette without installing global or per-widget QSS.
// Recursive mode is intended for a newly constructed panel or dialog.
void ApplyWidgetTheme(QWidget* widget, style::Tone tone = style::Tone::Shell,
                      bool recursive = true,
                      ThemeAppearance appearance = ThemeAppearance::kSystem);

}  // namespace autoviz::widget

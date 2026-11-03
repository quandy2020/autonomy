/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QPalette>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QWidget;

namespace autoviz::widget {

// Builds a Qt palette from the semantic Autoviz tokens. This is the styling
// bridge for native model/view, editor, date/time, slider, and combo widgets.
QPalette BuildWidgetPalette(style::Tone tone = style::Tone::Shell,
                            const QPalette& base = QPalette());

// Applies the semantic palette without installing global or per-widget QSS.
// Recursive mode is intended for a newly constructed panel or dialog.
void ApplyWidgetTheme(QWidget* widget, style::Tone tone = style::Tone::Shell,
                      bool recursive = true);

}  // namespace autoviz::widget

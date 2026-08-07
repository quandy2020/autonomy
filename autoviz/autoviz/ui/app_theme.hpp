/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>

class QApplication;
class QPalette;

namespace autoviz {

/** Qt Fusion highlight accent — matches RViz2 default shell. */
QColor AppThemeAccentColor();
/** Default 3D viewport background (48,48,48) — same as RViz2 visualization_manager. */
QColor AppThemeSuggestedViewportBackground();

QPalette BuildThemePalette();
/**
 * Apply RViz2-style theme: Fusion base style + light palette + minimal global QSS.
 *
 * Layering:
 *  1. QPalette — semantic colors (Fusion light)
 *  2. QSS — spacing, borders, panel #objectName rules
 */
void ApplyAppTheme(QApplication& app);
QString BuildAppThemeStylesheet();
/** Checkbox-style indicators for Panels menu (applied per QMenu via setStyleSheet). */
QString BuildPanelsMenuStylesheet();

}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file grid_map_color_maps.hpp
 * @brief Named colormaps for GridMap intensity → RGBA mapping.
 *
 * Used by @ref GridMapDisplay when @c use_colormap is enabled. Names are
 * listed by @ref GridMapColorMapNames for the property combo.
 *
 * @see GridMapDisplay
 * @see SampleGridMapColorMap
 */

#pragma once

#include <string>
#include <vector>

#include <QColor>

namespace autoviz {
namespace display {

/**
 * @brief Samples a named colormap at normalized intensity @p t ∈ [0, 1].
 *
 * @param name Colormap id (e.g. @c "default", @c "jet"); unknown names fall
 *        back to a built-in default as implemented in .cpp.
 * @param t Normalized intensity, typically clamped to [0, 1].
 * @return Interpolated @c QColor (alpha usually opaque).
 *
 * @see GridMapColorMapNames
 * @see GridMapDisplay
 */
QColor SampleGridMapColorMap(const std::string& name, float t);

/**
 * @brief Returns the list of colormap names offered in the property UI.
 *
 * @return Stable ordered vector of colormap id strings.
 *
 * @see SampleGridMapColorMap
 */
std::vector<std::string> GridMapColorMapNames();

}  // namespace display
}  // namespace autoviz

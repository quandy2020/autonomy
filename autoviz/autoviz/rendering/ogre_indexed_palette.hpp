/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_indexed_palette.hpp
 * @brief 256-entry rainbow palette for indexed intensity coloring.
 *
 * Aligned with rviz @c Indexed8BitImage intensity → color mapping used by
 * laser / point-cloud displays that color by intensity index.
 *
 * @see OgrePointCloud
 * @see ensureRainbowPalette()
 */

#pragma once

#include <array>
#include <cstdint>

#include <QColor>

namespace autoviz {
namespace rendering {

/**
 * @class OgreIndexedPalette
 * @brief Rainbow palette (256×1) for intensity-to-color lookup.
 *
 * All methods are static; call @ref ensureRainbowPalette() once before
 * reading @ref rainbowTable() or @ref colorFromIndex().
 */
class OgreIndexedPalette {
 public:
  /**
   * @brief Builds the shared 256-color rainbow table if not already done.
   *
   * Idempotent; safe to call from display init paths.
   */
  static void ensureRainbowPalette();

  /**
   * @brief Returns the shared 256-entry rainbow table.
   *
   * @return Const reference to the palette; valid after @ref ensureRainbowPalette().
   */
  static const std::array<QColor, 256>& rainbowTable();

  /**
   * @brief Maps a scalar intensity into a palette index.
   *
   * Linearly maps @p intensity from \[@p min_i, @p max_i\] into \[0, 255\].
   *
   * @param intensity Sample intensity value.
   * @param min_i Range minimum (inclusive).
   * @param max_i Range maximum (inclusive); if equal to @p min_i, returns 0.
   * @return Palette index in \[0, 255\].
   */
  static uint8_t intensityToIndex(float intensity, float min_i, float max_i);

  /**
   * @brief Looks up a palette color by index.
   *
   * @param index Palette entry (0–255).
   * @return Corresponding @c QColor from @ref rainbowTable().
   */
  static QColor colorFromIndex(uint8_t index);
};

}  // namespace rendering
}  // namespace autoviz


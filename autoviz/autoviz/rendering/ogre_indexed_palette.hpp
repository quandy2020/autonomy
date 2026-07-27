/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <array>
#include <cstdint>

#include <QColor>

namespace autoviz {
namespace rendering {

/** Rainbow palette (256×1) aligned with rviz/Indexed8BitImage intensity coloring. */
class OgreIndexedPalette {
 public:
  static void ensureRainbowPalette();

  /** Shared 256-entry table; valid after ensureRainbowPalette(). */
  static const std::array<QColor, 256>& rainbowTable();

  static uint8_t intensityToIndex(float intensity, float min_i, float max_i);
  static QColor colorFromIndex(uint8_t index);
};

}  // namespace rendering
}  // namespace autoviz

#endif

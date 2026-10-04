/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file pick_handle.hpp
 * @brief GPU/CPU pick handle types and RGB encoding (RViz-compatible).
 *
 * A @ref PickHandle identifies a selectable object across one render frame.
 * Handles are encoded as 24-bit RGB colors for GPU color-buffer picking;
 * alpha is ignored.
 *
 * @see PickRegistry
 * @see SelectionHandler
 * @see HandlerManager
 */

#pragma once

#include <cstdint>

namespace autoviz {
namespace common {

/**
 * @brief Opaque 32-bit pick identifier (0 is reserved as invalid).
 */
using PickHandle = uint32_t;

/**
 * @brief Sentinel for “no pick” / unassigned handle.
 */
constexpr PickHandle kInvalidPickHandle = 0;

/**
 * @struct PickColor
 * @brief RGB triple used to encode a @ref PickHandle in the pick framebuffer.
 *
 * RViz-compatible: only the lower 24 bits of the handle are stored.
 */
struct PickColor {
  uint8_t r = 0;  /**< Red channel (bits 16–23 of the handle). */
  uint8_t g = 0;  /**< Green channel (bits 8–15 of the handle). */
  uint8_t b = 0;  /**< Blue channel (bits 0–7 of the handle). */
};

/**
 * @brief Encodes a pick handle into an RGB color for GPU picking.
 *
 * @param handle Handle to encode; @ref kInvalidPickHandle yields black.
 * @return Corresponding @ref PickColor.
 * @see pickColorToHandle()
 */
PickColor handleToPickColor(PickHandle handle);

/**
 * @brief Decodes an RGB sample from the pick buffer into a handle.
 *
 * @param r Red channel (0–255).
 * @param g Green channel (0–255).
 * @param b Blue channel (0–255).
 * @return Reconstructed @ref PickHandle (may be @ref kInvalidPickHandle).
 * @see handleToPickColor()
 */
PickHandle pickColorToHandle(uint8_t r, uint8_t g, uint8_t b);

/**
 * @class PickHandleAllocator
 * @brief Monotonically allocates unique non-zero pick handles for one frame.
 *
 * Typically reset each draw via @ref reset() so handles stay dense and
 * within the 24-bit RGB range for longer sessions.
 */
class PickHandleAllocator {
 public:
  /**
   * @brief Allocates the next unused handle.
   *
   * @return New handle (≥ 1). Does not wrap; callers should @ref reset()
   *         each frame.
   */
  PickHandle allocate();

  /**
   * @brief Resets the sequence so the next @ref allocate() returns @c 1.
   */
  void reset() { next_handle_ = 1; }

 private:
  /** Next handle to return from @ref allocate(). */
  PickHandle next_handle_ = 1;
};

}  // namespace common
}  // namespace autoviz

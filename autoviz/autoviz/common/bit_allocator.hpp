/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file bit_allocator.hpp
 * @brief Single-bit slot allocator for 32-bit display visibility masks.
 *
 * Mirrors @c rviz_common::BitAllocator. Each display (or renderable group) can
 * claim one bit so viewport filtering can show/hide subsets via a mask.
 *
 * @see DisplayContext::default_visibility_bit
 */

#pragma once

#include <cstdint>

namespace autoviz {
namespace common {

/**
 * @class BitAllocator
 * @brief Allocates unused single bits within a 32-bit integer mask.
 *
 * @note When all 32 bits are taken, @ref allocBit() returns @c 0 (also the
 *       value of the unused sentinel). Callers must treat @c 0 as failure.
 */
class BitAllocator {
 public:
  /**
   * @brief Constructs an allocator with no bits reserved.
   */
  BitAllocator();

  /**
   * @brief Claims the next free bit in the mask.
   *
   * @return A power-of-two bit value (1, 2, 4, …), or @c 0 if none remain.
   */
  uint32_t allocBit();

  /**
   * @brief Releases one or more previously allocated bits.
   *
   * @param bits Bitmask of bits to free (OR of values returned by
   *        @ref allocBit()). Bits that were never allocated are ignored.
   */
  void freeBits(uint32_t bits);

 private:
  /** Bitmask of currently allocated bits (1 = in use). */
  uint32_t allocated_bits_ = 0;
};

}  // namespace common
}  // namespace autoviz

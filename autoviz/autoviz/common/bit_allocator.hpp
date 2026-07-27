/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cstdint>

namespace autoviz {
namespace common {

/** rviz_common::BitAllocator — single-bit slots in a 32-bit visibility mask. */
class BitAllocator {
 public:
  BitAllocator();

  /** Returns one unused bit, or 0 if all 32 bits are allocated. */
  uint32_t allocBit();

  void freeBits(uint32_t bits);

 private:
  uint32_t allocated_bits_ = 0;
};

}  // namespace common
}  // namespace autoviz

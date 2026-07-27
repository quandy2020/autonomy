/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/bit_allocator.hpp"

namespace autoviz {
namespace common {

BitAllocator::BitAllocator() = default;

uint32_t BitAllocator::allocBit() {
  uint32_t mask = 1;
  for (int i = 0; i < 32; ++i) {
    if ((mask & allocated_bits_) == 0) {
      allocated_bits_ |= mask;
      return mask;
    }
    mask <<= 1;
  }
  return 0;
}

void BitAllocator::freeBits(uint32_t bits) {
  allocated_bits_ &= ~bits;
}

}  // namespace common
}  // namespace autoviz

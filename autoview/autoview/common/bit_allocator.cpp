/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autoview/common/bit_allocator.hpp"

namespace autoview {
namespace common {

BitAllocator::BitAllocator()
: allocated_bits_(0)
{}

uint32_t BitAllocator::allocBit()
{
  uint32_t mask = 1;
  for (int i = 0; i < 32; i++) {
    if ( (mask & allocated_bits_) == 0) {
      allocated_bits_ |= mask;
      return mask;
    }

    mask <<= 1;
  }
  return 0;
}

void BitAllocator::freeBits(uint32_t bits)
{
  allocated_bits_ &= ~bits;
}

}  // namespace common
}  // namespace autoview

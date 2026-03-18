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

 #pragma once

#include <cstdint>

namespace autoview {
namespace common {

/// Allocation manager for bit positions within a 32-bit word.
class BitAllocator
{
public:
  /// Constructor; all bits are free initially.
  BitAllocator();

  /// Return a uint32 with a single unused bit "on", or a 0 if all bits are already allocated.
  uint32_t allocBit();

  /// Free the given bits.
  void freeBits(uint32_t bits);

private:
  uint32_t allocated_bits_;
};

}  // namespace common
}  // namespace autoview
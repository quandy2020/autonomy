/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/pick_handle.hpp"

namespace autoviz {
namespace common {

PickColor handleToPickColor(PickHandle handle) {
  PickColor color;
  color.r = static_cast<uint8_t>((handle >> 16) & 0xFF);
  color.g = static_cast<uint8_t>((handle >> 8) & 0xFF);
  color.b = static_cast<uint8_t>(handle & 0xFF);
  return color;
}

PickHandle pickColorToHandle(uint8_t r, uint8_t g, uint8_t b) {
  return (static_cast<PickHandle>(r) << 16) |
         (static_cast<PickHandle>(g) << 8) | static_cast<PickHandle>(b);
}

PickHandle PickHandleAllocator::allocate() {
  if (next_handle_ == kInvalidPickHandle) {
    ++next_handle_;
  }
  if (next_handle_ >= 0xFFFFFF) {
    next_handle_ = 1;
  }
  return next_handle_++;
}

}  // namespace common
}  // namespace autoviz

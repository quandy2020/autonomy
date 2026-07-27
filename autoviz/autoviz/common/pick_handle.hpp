/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cstdint>

namespace autoviz {
namespace common {

using PickHandle = uint32_t;

constexpr PickHandle kInvalidPickHandle = 0;

/** RViz-compatible 24-bit handle encoded as RGB (alpha ignored). */
struct PickColor {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

PickColor handleToPickColor(PickHandle handle);
PickHandle pickColorToHandle(uint8_t r, uint8_t g, uint8_t b);

class PickHandleAllocator {
 public:
  PickHandle allocate();

  void reset() { next_handle_ = 1; }

 private:
  PickHandle next_handle_ = 1;
};

}  // namespace common
}  // namespace autoviz
